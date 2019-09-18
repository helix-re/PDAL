/******************************************************************************
 * Copyright (c) 2019, Helix Re Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following
 * conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Helix Re Inc. nor the
 *       names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior
 *       written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 ****************************************************************************/

#include "E57Reader.hpp"
#include "Utilities.hpp"

namespace pdal
{

static PluginInfo const s_info{"readers.e57", "Reader for E57 files", ""};

CREATE_SHARED_STAGE(E57Reader, s_info)

std::string E57Reader::getName() const
{
    return s_info.name;
}

E57Reader::E57Reader()
    : Reader(), Streamable(), m_currentScan(-1), m_currentIndex(0),
      m_pointsInCurrentBatch(0), m_defaultChunkSize(1000000)
{
}

void E57Reader::addDimensions(PointLayoutPtr layout)
{
    for (auto& keyVal : m_doubleBuffers)
    {
        layout->registerDim(e57plugin::e57ToPdal(keyVal.first));
    }
}

void E57Reader::initialize()
{
    try
    {
        m_imf.reset(new ImageFile(m_filename, "r"));
        StructureNode root = m_imf->root();

        if (!root.isDefined("/data3D"))
        {
            throwError("File doesn't contain 3D data");
        }

        const e57::ustring normalsExtension(
            "http://www.libe57.org/E57_NOR_surface_normals.txt");
        e57::ustring _normalsExtension;

        // the extension may already be registered
        if (!m_imf->extensionsLookupPrefix("nor", _normalsExtension))
            m_imf->extensionsAdd("nor", normalsExtension);

        m_data3D.reset(new VectorNode(root.get("/data3D")));

        StructureNode scan(m_data3D->get(0));
        CompressedVectorNode points(scan.get("points"));
        m_e57PointPrototype.reset(new StructureNode(points.prototype()));

        auto supportedFields = e57plugin::supportedE57Types();
        for (auto& dimension : supportedFields)
        {
            if (m_e57PointPrototype->isDefined(dimension))
                m_doubleBuffers[dimension] =
                    std::vector<double>(m_defaultChunkSize, 0);
        }
    }
    catch (E57Exception& e)
    {
        throwError(std::to_string(e.errorCode()) + " : " + e.context());
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }
}

void E57Reader::ready(PointTableRef& ref)
{
    for (auto& keyValue : m_doubleBuffers)
    {
        m_destBuffers.emplace_back(
            *m_imf, keyValue.first, keyValue.second.data(), m_defaultChunkSize,
            true,
            (m_e57PointPrototype->get(keyValue.first).type() ==
             e57::E57_SCALED_INTEGER));
    }

    // Initial reader setup.
    setupReader();
}

/// Setup reader to read next scan if available.
void E57Reader::setupReader()
{
    // Are we done with reading all scans?
    if (++m_currentScan >= m_data3D->childCount())
        return;

    try
    {
        StructureNode scan(m_data3D->get(m_currentScan));

        // Clear rescale factors for previous scan, If any.
        m_rescaleFactors.clear();

        // Get rescale factors for new scan, These factors are only for
        // colors and intensity. This is required Since,
        // 1. E57 support colors in uint8 as well as uint16. This is specified
        //    in header as "colorLimits".
        //		- If colorLimits is 0-255 then it is uint8 and if 0-65535 then it
        // is uint16.
        //		- To make this consistant, We are rescaling colors to 0-65535
        //        range, Since default types for colors in PDAL are Unsigned16.
        // 2. E57 supports intensity in float ranging 0-1.  This is specified in
        //    header as "intensityLimits".
        //		- Since default type for intensity in PDAL is Unsigned16, E57
        //        intensity (between 0-1) need to be rescaled in uint16 (between
        //        0-65535).
        // To do the rescling we need the rescale factors so that we can directly
        // multipy them with colors and intensity values.
        // E.g. - If color limit is 0-255 then rescale factor would be 257.00
        //        (double value) i.e 65535/(255-0)=257.
        //		- If color limit is 0-65535 then rescale factor would be 1.00
        //        (double value) i.e 65535/(65535-0)= 1.
        for (auto& keyValue : m_doubleBuffers)
        {
            auto minmax = std::make_pair(0.0, 0.0);
            if (e57plugin::getLimits(scan, keyValue.first, minmax))
            {
                m_rescaleFactors[e57plugin::e57ToPdal(keyValue.first)] =
                    (std::numeric_limits<uint16_t>::max)() /
                    (minmax.second - minmax.first);
            }
        }

        CompressedVectorNode points(scan.get("points"));
        m_hasPose = e57plugin::getPose(scan, m_rotation, m_translation);
        m_reader.reset(
            new CompressedVectorReader(points.reader(m_destBuffers)));
    }
    catch (E57Exception& e)
    {
        throwError(std::to_string(e.errorCode()) + " : " + e.context());
    }
    catch (...)
    {
        throwError("Got an unknown exception");
    }
}

/// Read the next batch of m_defaultChunkSize.
/// This returns number of points aquired.
/// Returns 0 after finished reading of all scans.
point_count_t E57Reader::readNextBatch()
{
    m_currentIndex = 0;

    // Are we done with reading all scans?
    if (m_currentScan >= m_data3D->childCount())
        return 0;

    point_count_t gotPoints = m_reader->read(m_destBuffers);

    if (!gotPoints)
    {
        // Finished reading all points in current scan.
        // Its time to setup reader at next scan.
        m_reader->close();
        setupReader();
        return readNextBatch();
    }

    return gotPoints;
}

/// Fill the point information.
bool E57Reader::fillPoint(PointRef& point)
{
    if (m_currentIndex >= m_pointsInCurrentBatch)
    {
        // Either we are at very begining or finished processing all points in
        // current batch. Its time to read new points batch.
        m_pointsInCurrentBatch = readNextBatch();
    }

    if (!m_pointsInCurrentBatch)
    {
        // We're done with reding
        return false;
    }

    for (auto& keyValue : m_doubleBuffers)
    {
        auto dim = e57plugin::e57ToPdal(keyValue.first);

        if (dim != Dimension::Id::Unknown)
        {
            // Need to rescale?
            if (m_rescaleFactors.find(dim) != m_rescaleFactors.end())
                point.setField(dim, keyValue.second[m_currentIndex] *
                                        m_rescaleFactors[dim]);
            else
                point.setField(dim, keyValue.second[m_currentIndex]);
        }
    }

    if (m_hasPose)
        e57plugin::transformPoint(point, m_rotation, m_translation);

    ++m_currentIndex;
    return true;
}

point_count_t E57Reader::read(PointViewPtr view, point_count_t count)
{
    point_count_t numPoints = e57plugin::numPoints(*m_data3D);
    for (PointId counter = 0, nextId = view->size(); counter < numPoints;
         ++counter, ++nextId)
    {
        fillPoint(view->point(nextId));
    }

    return view->size();
}

bool E57Reader::processOne(PointRef& point)
{
    return fillPoint(point);
}

void E57Reader::done(PointTableRef table)
{
    m_imf->close();
}

} // namespace pdal