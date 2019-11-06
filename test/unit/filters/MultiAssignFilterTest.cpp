/******************************************************************************
 * Copyright (c) 2019, Helix Re Inc. <pravin@helix.re>
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
 *     * Neither the name of Helix Re, Inc. nor the
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

#include <pdal/pdal_test_main.hpp>

#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include "nlohmann/json.hpp"

#include "Support.hpp"

using namespace pdal;

TEST(MultiAssignFilterTest, test_value)
{
    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-dd.las"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);

    Options fo;
    NL::json json =
        "{\"assignments\" : [{\"assign\" : \"Red[:]=255,Green[:]=255,Blue[:]=255\"}]}"_json;
    fo.add("args", json);

    Stage& f = *(factory.createStage("filters.multiassign"));
    f.setInput(r);
    f.setOptions(fo);

    std::string tempfile(Support::temppath("out.las"));

    Options wo;
    wo.add("filename", tempfile);
    Stage& w = *(factory.createStage("writers.las"));
    w.setInput(f);
    w.setOptions(wo);

    FileUtils::deleteFile(tempfile);
    PointTable t1;
    w.prepare(t1);
    w.execute(t1);

    Options testOptions;
    testOptions.add("filename", tempfile);

    Stage& test = *(factory.createStage("readers.las"));
    test.setOptions(testOptions);

    PointTable t2;
    test.prepare(t2);
    PointViewSet s = test.execute(t2);
    PointViewPtr v = *s.begin();
    for (PointId i = 0; i < v->size(); ++i)
    {
        EXPECT_EQ(v->getFieldAs<int>(Dimension::Id::Red, i), 255);
        EXPECT_EQ(v->getFieldAs<int>(Dimension::Id::Green, i), 255);
        EXPECT_EQ(v->getFieldAs<int>(Dimension::Id::Blue, i), 255);
    }
}
TEST(MultiAssignFilterTest, test_dim_ranges)
{
    StageFactory factory;

    Stage& r = *factory.createStage("readers.las");
    Stage& f = *factory.createStage("filters.multiassign");

    // utm17.las contains 5 points with intensity of 280, 3 of 260 and 2 of 240
    Options ro;
    ro.add("filename", Support::datapath("las/utm17.las"));
    r.setOptions(ro);

    Options fo;
    NL::json json =
        "{\"assignments\" : [{\"assign\" : \"Intensity[:250]=4,Intensity[245:270 ]=6,Intensity[272:] = 8\"}]}"_json;
    fo.add("args", json);

    f.setInput(r);
    f.setOptions(fo);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);
    PointViewPtr v = *s.begin();

    int i4 = 0;
    int i6 = 0;
    int i8 = 0;
    for (PointId i = 0; i < v->size(); ++i)
    {
        int ii = v->getFieldAs<int>(Dimension::Id::Intensity, i);
        if (ii == 4)
            i4++;
        else if (ii == 6)
            i6++;
        else if (ii == 8)
            i8++;
    }
    EXPECT_EQ(i4, 2);
    EXPECT_EQ(i6, 3);
    EXPECT_EQ(i8, 5);
}

TEST(MultiAssignFilterTest, test_condition)
{
    StageFactory factory;

    Stage& r = *factory.createStage("readers.las");
    Stage& f = *factory.createStage("filters.multiassign");

    // utm17.las contains 5 points with intensity of 280, 3 of 260 and 2 of 240
    Options ro;
    ro.add("filename", Support::datapath("las/utm17.las"));
    r.setOptions(ro);

    Options fo;
    NL::json json =
        "{\"assignments\" : [{\"assign\" : \"PointSourceId[:]=6\",\"condition\":\"Intensity[260:260]\"}]}"_json;
    fo.add("args", json);

    f.setInput(r);
    f.setOptions(fo);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);
    PointViewPtr v = *s.begin();

    int ielse = 0;
    int i6 = 0;
    for (PointId i = 0; i < v->size(); ++i)
    {
        int ii = v->getFieldAs<int>(Dimension::Id::PointSourceId, i);
        if (ii == 6)
            i6++;
        else
            ielse++;
    }
    EXPECT_EQ(i6, 3);
    EXPECT_EQ(v->size(), 10u);
    EXPECT_EQ(ielse, 7);
}

TEST(MultiAssignFilterTest, test_multiple_assignments)
{
    StageFactory factory;

    Stage& r = *factory.createStage("readers.las");
    Stage& f = *factory.createStage("filters.multiassign");

    // utm17.las contains 5 points with intensity of 280, 3 of 260 and 2 of 240
    Options ro;
    ro.add("filename", Support::datapath("las/utm17.las"));
    r.setOptions(ro);

    Options fo;
    NL::json json =
        "{\"assignments\" : [{\"assign\" : \"PointSourceId[:]=6\",\"condition\":\"Intensity[260:260]\"},{\"assign\" : \"PointSourceId[:]=8\",\"condition\":\"Intensity[261:]\"},{\"assign\" : \"PointSourceId[:]=4\",\"condition\":\"Intensity[240:259]\"}]}"_json;
    fo.add("args", json);

    f.setInput(r);
    f.setOptions(fo);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);
    PointViewPtr v = *s.begin();

    int ielse = 0;
    int i6 = 0, i8=0, i4=0;
    for (PointId i = 0; i < v->size(); ++i)
    {
        int ii = v->getFieldAs<int>(Dimension::Id::PointSourceId, i);
        if (ii == 6)
            i6++;
        else if (ii == 8)
            i8++;
        else if (ii == 4)
            i4++;
        else
            ielse++;
    }
    EXPECT_EQ(i6, 3);
    EXPECT_EQ(i8, 5);
    EXPECT_EQ(i4, 2);
    EXPECT_EQ(v->size(), 10u);
    EXPECT_EQ(ielse, 0);
}