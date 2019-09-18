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

#include "Support.hpp"
#include "plugins/e57/io/E57Reader.hpp"
#include "plugins/e57/io/Utilities.hpp"
#include <gtest/gtest.h>

using namespace pdal::e57plugin;
using namespace pdal;

TEST(E57Utiliies, e57ToPdalTranslation)
{
    ASSERT_EQ(e57ToPdal("cartesianX"), pdal::Dimension::Id::X);
    ASSERT_EQ(e57ToPdal("fake"), pdal::Dimension::Id::Unknown);
    ASSERT_EQ(e57ToPdal("cartesianY"), pdal::Dimension::Id::Y);
    ASSERT_EQ(e57ToPdal("cartesianZ"), pdal::Dimension::Id::Z);
    ASSERT_EQ(e57ToPdal("colorRed"), pdal::Dimension::Id::Red);
    ASSERT_EQ(e57ToPdal("colorGreen"), pdal::Dimension::Id::Green);
    ASSERT_EQ(e57ToPdal("colorBlue"), pdal::Dimension::Id::Blue);
    ASSERT_EQ(e57ToPdal("intensity"), pdal::Dimension::Id::Intensity);
}

TEST(E57Utiliies, transformPoint)
{
    Options ops;
    ops.add("filename", Support::datapath("e57/A4.e57"));
    E57Reader reader;
    reader.setOptions(ops);
    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    auto cloud = *viewSet.begin();
    auto point = cloud->point(0);
    point.setField(pdal::Dimension::Id::X, 1.0);
    point.setField(pdal::Dimension::Id::Y, 2.0);
    point.setField(pdal::Dimension::Id::Z, 3.0);
    double rot[3][3] = {{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    double trans[3] = {2, 2, 2};
    transformPoint(point, rot, trans);
    ASSERT_EQ(point.getFieldAs<double>(pdal::Dimension::Id::X), 8.0f);
    ASSERT_EQ(point.getFieldAs<double>(pdal::Dimension::Id::Y), 14.0f);
    ASSERT_EQ(point.getFieldAs<double>(pdal::Dimension::Id::Z), 20.0f);
}

TEST(E57Utiliies, numPoints)
{
    e57::ImageFile imf(Support::datapath("e57/A4.e57"), "r");
    auto data3D = new e57::VectorNode(imf.root().get("/data3D"));
    ASSERT_EQ(numPoints(*data3D), 4u);
    imf.close();
}

TEST(E57Utiliies, getLimits)
{
    e57::ImageFile imf(Support::datapath("e57/A4.e57"), "r");
    auto data3D = new e57::VectorNode(imf.root().get("/data3D"));
    auto pair = std::make_pair(0.0, 0.0);
    auto scan = new e57::StructureNode(data3D->get(0));
    ASSERT_TRUE(getLimits(*scan, "colorRed", pair));
    ASSERT_EQ(pair.first, 0.0);
    ASSERT_EQ(pair.second, 255.0);

    ASSERT_TRUE(getLimits(*scan, "intensity", pair));
    ASSERT_EQ(pair.first, 0.0);
    ASSERT_EQ(pair.second, 0.0);
    imf.close();
}
