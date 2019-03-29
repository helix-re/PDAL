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

#include "Scan.hpp"
#include "Utils.hpp"

namespace e57
{

    Scan::Scan(const e57::StructureNode &node) : m_numPoints(0)
{
    m_rawData = std::unique_ptr<e57::StructureNode>(new e57::StructureNode(node));
    m_rawPoints = std::unique_ptr<CompressedVectorNode>(new CompressedVectorNode(m_rawData->get("points")));
    decodeHeader();
}

pdal::point_count_t Scan::getNumPoints() const
{
    return m_numPoints;
}

std::set<std::string> Scan::getDimensions() const
{
    return m_e57TypeToPdalDimension;
}

e57::CompressedVectorNode Scan::getPoints() const
{
    return *m_rawPoints;
}

std::pair<double,double> Scan::getLimits(pdal::Dimension::Id pdalId) const
{
    return m_valueBounds.at(pdalId);
}

bool Scan::hasPose() const
{
    return m_hasPose;
}

void Scan::transformPoint(pdal::PointRef pt) const
{
    auto x = pt.getFieldAs<double>(pdal::Dimension::Id::X);
    auto y = pt.getFieldAs<double>(pdal::Dimension::Id::Y);
    auto z = pt.getFieldAs<double>(pdal::Dimension::Id::Z);

    pt.setField(pdal::Dimension::Id::X, x*m_rotation[0][0] + y*m_rotation[0][1] + z*m_rotation[0][2] + m_translation[0]);
    pt.setField(pdal::Dimension::Id::Y,  x*m_rotation[1][0] + y*m_rotation[1][1] + z*m_rotation[1][2]  + m_translation[1]);
    pt.setField(pdal::Dimension::Id::Z,  x*m_rotation[2][0] + y*m_rotation[2][1] + z*m_rotation[2][2]  + m_translation[2]);
}

    void Scan::decodeHeader()
{
    m_numPoints = m_rawPoints->childCount();

    auto supportedFields = pdal::e57plugin::supportedE57Types();
    e57::StructureNode prototype(m_rawPoints->prototype());
    
    // Extract fields that can be extracted
    for (auto& field: supportedFields)
    {
        if (prototype.isDefined(field))
        {
            m_e57TypeToPdalDimension.insert(field);
        }
    }
    // Get pose estimation
    getPose();

    // Get bounds
    for (auto& field: supportedFields)
    {
        auto minmax = pdal::e57plugin::getLimits(*m_rawData,field);
        if (minmax == minmax) // not nan
        {
            m_valueBounds[pdal::e57plugin::e57ToPdal(field)] = minmax;
        }
    }
}

    void Scan::getPose()
{
    if (m_rawData->isDefined("pose"))
	{
        m_rotation[0][0] = 1; m_rotation[1][1] = 1; m_rotation[2][2] = 1;
		e57::StructureNode pose(m_rawData->get("pose"));
		if (pose.isDefined("rotation"))
		{
			e57::StructureNode rotNode(pose.get("rotation"));
			double q[4];
			q[0] = e57::FloatNode(rotNode.get("w")).value();
			q[1] = e57::FloatNode(rotNode.get("x")).value();
			q[2] = e57::FloatNode(rotNode.get("y")).value();
			q[3] = e57::FloatNode(rotNode.get("z")).value();

			double q11 = q[1] * q[1];
			double q22 = q[2] * q[2];
			double q33 = q[3] * q[3];
			double q03 = q[0] * q[3];
			double q13 = q[1] * q[3];
			double q23 = q[2] * q[3];
			double q02 = q[0] * q[2];
			double q12 = q[1] * q[2];
			double q01 = q[0] * q[1];

            // 			m_rotation[0][0] = q00 + q11 - q22 - q33;
			// m_rotation[1][1] = q00 - q11 + q22 - q33;
			// m_rotation[2][2] = q00 - q11 - q22 + q33;
			m_rotation[0][0] = 1 - 2.0*(q22 + q33);
			m_rotation[1][1] = 1 - 2.0*(q11 + q33);
			m_rotation[2][2] = 1 - 2.0*(q11 + q22);
			m_rotation[0][1] = 2.0*(q12 - q03);
			m_rotation[1][0] = 2.0*(q12 + q03);
			m_rotation[0][2] = 2.0*(q13 + q02);
			m_rotation[2][0] = 2.0*(q13 - q02);
			m_rotation[1][2] = 2.0*(q23 - q01);
            m_rotation[2][1] = 2.0*(q23 + q01);
			m_hasPose = true;
		}

		if (pose.isDefined("translation"))
		{
			e57::StructureNode transNode(pose.get("translation"));  
			m_translation[0] = e57::FloatNode(transNode.get("x")).value();
			m_translation[1] = e57::FloatNode(transNode.get("y")).value();
			m_translation[2] = e57::FloatNode(transNode.get("z")).value();
			m_hasPose = true;
		}
	}
}

}