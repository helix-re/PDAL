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

#include "Utilities.hpp"

namespace pdal
{
namespace e57plugin
{
Dimension::Id e57ToPdal(const std::string& e57Dimension)
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

void transformPoint(PointRef& pt, const double rotation[3][3],
                              const double translation[3])
{
    auto x = pt.getFieldAs<double>(pdal::Dimension::Id::X);
    auto y = pt.getFieldAs<double>(pdal::Dimension::Id::Y);
    auto z = pt.getFieldAs<double>(pdal::Dimension::Id::Z);

    pt.setField(pdal::Dimension::Id::X,
                x * rotation[0][0] + y * rotation[0][1] + z * rotation[0][2] +
                    translation[0]);
    pt.setField(pdal::Dimension::Id::Y,
                x * rotation[1][0] + y * rotation[1][1] + z * rotation[1][2] +
                    translation[1]);
    pt.setField(pdal::Dimension::Id::Z,
                x * rotation[2][0] + y * rotation[2][1] + z * rotation[2][2] +
                    translation[2]);
}
std::vector<std::string> supportedE57Types()
{
    return {"cartesianX",  "cartesianY",           "cartesianZ",
            "nor:normalX", "nor:normalY",          "nor:normalZ",
            "colorRed",    "colorGreen",           "colorBlue",
            "intensity",   "cartesianInvalidState"};
}

bool getPose(const e57::StructureNode scan, double (&rotation)[3][3],
                       double (&translation)[3])
{
    bool hasPose = false;
    if (scan.isDefined("pose"))
    {
		// Reset rotation and translation.
        rotation[0][0] = 1; rotation[0][1] = 0;	rotation[0][2] = 0;
        rotation[1][0] = 0; rotation[1][1] = 1;	rotation[1][2] = 0;
        rotation[2][0] = 0; rotation[2][1] = 0;	rotation[2][2] = 1;
        translation[0] = 0; translation[1] = 0; translation[2] = 0;

		hasPose=true;

        e57::StructureNode pose(scan.get("pose"));
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

            rotation[0][0] = 1 - 2.0 * (q22 + q33);
            rotation[1][1] = 1 - 2.0 * (q11 + q33);
            rotation[2][2] = 1 - 2.0 * (q11 + q22);
            rotation[0][1] = 2.0 * (q12 - q03);
            rotation[1][0] = 2.0 * (q12 + q03);
            rotation[0][2] = 2.0 * (q13 + q02);
            rotation[2][0] = 2.0 * (q13 - q02);
            rotation[1][2] = 2.0 * (q23 - q01);
            rotation[2][1] = 2.0 * (q23 + q01);
        }

        if (pose.isDefined("translation"))
        {
            e57::StructureNode transNode(pose.get("translation"));
            translation[0] = e57::FloatNode(transNode.get("x")).value();
            translation[1] = e57::FloatNode(transNode.get("y")).value();
            translation[2] = e57::FloatNode(transNode.get("z")).value();
        }
    }
    return hasPose;
}


point_count_t numPoints(const e57::VectorNode data3D)
{
    point_count_t count(0);
    int64_t scanCount=data3D.childCount();
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
            minmax.second = static_cast<e57::ScaledIntegerNode>(intbox.get(maxKey))
                         .scaledValue();
            minmax.first = static_cast<e57::ScaledIntegerNode>(intbox.get(minKey))
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

} // namespace e57plugin

} // namespace pdal