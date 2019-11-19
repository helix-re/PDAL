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

#include <limits>

#include "Utils.hpp"

namespace pdal
{
namespace e57plugin
{

Dimension::Id e57ToPdal(const std::string &e57Dimension)
{
    if (e57Dimension == "cartesianX")
        return Dimension::Id::X;
    else if (e57Dimension == "cartesianY")
        return Dimension::Id::Y;
    else if (e57Dimension == "cartesianZ")
        return Dimension::Id::Z;
    else if (e57Dimension == "sphericalRange")
        return Dimension::Id::X;
    else if (e57Dimension == "sphericalAzimuth")
        return Dimension::Id::Y;
    else if (e57Dimension == "sphericalElevation")
        return Dimension::Id::Z;
    else if (e57Dimension == "nor:normalX")
        return Dimension::Id::NormalX;
    else if (e57Dimension == "nor:normalY")
        return Dimension::Id::NormalY;
    else if (e57Dimension == "nor:normalZ")
        return Dimension::Id::NormalZ;
    else if (e57Dimension == "intensity")
        return Dimension::Id::Intensity;
    else if (e57Dimension == "colorRed")
        return Dimension::Id::Red;
    else if (e57Dimension == "colorBlue")
        return Dimension::Id::Blue;
    else if (e57Dimension == "colorGreen")
        return Dimension::Id::Green;
    else if (e57Dimension == "cartesianInvalidState")
        return Dimension::Id::Omit;
    else if (e57Dimension == "sphericalInvalidState")
        return Dimension::Id::Omit;
    return Dimension::Id::Unknown;
}

std::string pdalToE57(Dimension::Id pdalDimension)
{
    switch (pdalDimension)
    {
        case pdal::Dimension::Id::X:
            return "cartesianX";
        case pdal::Dimension::Id::Y:
            return "cartesianY";
        case pdal::Dimension::Id::Z:
            return "cartesianZ";
        case pdal::Dimension::Id::NormalX:
            return "nor:normalX";
        case pdal::Dimension::Id::NormalY:
            return "nor:normalY";
        case pdal::Dimension::Id::NormalZ:
            return "nor:normalZ";
        case pdal::Dimension::Id::Red:
            return "colorRed";
        case pdal::Dimension::Id::Green:
            return "colorGreen";
        case pdal::Dimension::Id::Blue:
            return "colorBlue";
        case pdal::Dimension::Id::Intensity:
            return "intensity";
        case pdal::Dimension::Id::Omit:
            return "cartesianInvalidState";
        default:
            return std::string();
    }
}

std::vector<Dimension::Id> supportedPdalTypes()
{
    return {Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z,
            Dimension::Id::NormalX, Dimension::Id::NormalY, Dimension::Id::NormalZ,
            Dimension::Id::Red, Dimension::Id::Green, Dimension::Id::Blue,
            Dimension::Id::Intensity, Dimension::Id::Omit
           };
}

std::vector<std::string> supportedE57Types()
{
    return {"cartesianX",  "cartesianY", "cartesianZ",
            "nor:normalX", "nor:normalY", "nor:normalZ",
            "colorRed", "colorGreen", "colorBlue", "intensity",
            "cartesianInvalidState"};
}

inline std::vector<std::string> rescalableE57Types()
{
    return {"colorRed", "colorGreen", "colorBlue", "intensity"};
}

bool getLimits(const e57::StructureNode& prototype,
               const std::string& fieldName, std::pair<double, double>& minmax)
{
    // Reset minmax
    minmax.first = minmax.second = 0;

    std::string minKey = fieldName + "Minimum";
    std::string maxKey = fieldName + "Maximum";
    std::string boundingBoxName = fieldName + "Limits";

    if (fieldName.substr(0, 5) == "color")
        boundingBoxName = "colorLimits";
    else if (fieldName[0] == 'x' || fieldName[0] == 'y' || fieldName[0] == 'z')
        boundingBoxName = "cartesianBounds";

    if (prototype.isDefined(boundingBoxName))
    {
        e57::StructureNode intbox(prototype.get(boundingBoxName));
        if (intbox.get(maxKey).type() == e57::E57_SCALED_INTEGER)
        {
            minmax.second =
                static_cast<e57::ScaledIntegerNode>(intbox.get(maxKey))
                .scaledValue();
            minmax.first =
                static_cast<e57::ScaledIntegerNode>(intbox.get(minKey))
                .scaledValue();
        }
        else if (intbox.get(maxKey).type() == e57::E57_FLOAT)
        {
            minmax.second =
                static_cast<e57::FloatNode>(intbox.get(maxKey)).value();
            minmax.first =
                static_cast<e57::FloatNode>(intbox.get(minKey)).value();
        }
        else if (intbox.get(maxKey).type() == e57::E57_INTEGER)
        {
            minmax.second = static_cast<double>(
                                static_cast<e57::IntegerNode>(intbox.get(maxKey)).value());
            minmax.first = static_cast<double>(
                               static_cast<e57::IntegerNode>(intbox.get(minKey)).value());
        }
    }
    else if (prototype.isDefined(fieldName))
    {
        if (prototype.get(fieldName).type() == e57::E57_INTEGER)
        {
            minmax.first = static_cast<double>(
                               static_cast<e57::IntegerNode>(prototype.get(fieldName))
                               .minimum());
            minmax.second = static_cast<double>(
                                static_cast<e57::IntegerNode>(prototype.get(fieldName))
                                .maximum());
        }
        else if (prototype.get(fieldName).type() == e57::E57_SCALED_INTEGER)
        {
            double scale =
                static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName))
                .scale();
            double offset =
                static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName))
                .offset();
            int64_t minimum =
                static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName))
                .minimum();
            int64_t maximum =
                static_cast<e57::ScaledIntegerNode>(prototype.get(fieldName))
                .maximum();
            minmax.first = minimum * scale + offset;
            minmax.second = maximum * scale + offset;
        }
        else if (prototype.get(fieldName).type() == e57::E57_FLOAT)
        {
            minmax.first =
                static_cast<e57::FloatNode>(prototype.get(fieldName)).minimum();
            minmax.second =
                static_cast<e57::FloatNode>(prototype.get(fieldName)).maximum();
        }
    }
    else
        return false;
    return true;
}

std::pair<uint64_t, uint64_t> getPdalBounds(pdal::Dimension::Id id)
{
    //ABELL - This is strange.  PDAL doesn't specify limits for these.
    //  Perhaps argument registration needs to be changed.
    using Dim = pdal::Dimension::Id;
    switch (id)
    {
        case Dim::Red:
        case Dim::Blue:
        case Dim::Green:
        case Dim::Intensity:
            return {std::numeric_limits<uint16_t>::min(),
                    std::numeric_limits<uint16_t>::max()};
        default:
            std::string msg ="Dimension " + Dimension::name(id) +
                             " is not currently supported.";
            throw pdal_error(msg);
    }
}

point_count_t numPoints(const e57::VectorNode data3D)
{
    point_count_t count(0);
    int64_t scanCount = data3D.childCount();
    try
    {
        for (int scanIndex = 0; scanIndex < scanCount; scanIndex++)
        {
            e57::StructureNode scan(data3D.get(scanIndex));
            e57::CompressedVectorNode points(scan.get("points"));
            count += points.childCount();
        }
    }
    catch (e57::E57Exception& e)
    {
        e.report(__FILE__, __LINE__, __FUNCTION__);
    }
    catch (...)
    {
        throw pdal_error("Got an unknown exception");
    }
    return count;
}

} // namespace e57plugin
} // namespace pdal

