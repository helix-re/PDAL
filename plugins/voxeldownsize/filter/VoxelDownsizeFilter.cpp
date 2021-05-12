/******************************************************************************
 * Copyright (c) 2019, Helix.re
 * Contact Person : Pravin Shinde (pravin@helix.re,
 *                    https://github.com/pravinshinde825)
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
 *     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
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

#include "VoxelDownsizeFilter.hpp"
#include <arbiter/arbiter.hpp>
#include "leveldb/cache.h"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.voxeldownsize",
    "First Entry Voxel Filter",
    "http://pdal.io/stages/filters.voxeldownsize.html"
};

CREATE_SHARED_STAGE(VoxelDownsizeFilter, s_info)

std::istream& operator>>(std::istream& in, VoxelDownsizeFilter::Mode& mode)
{
    std::string s;
    in >> s;

    s = Utils::tolower(s);
    if (s == "center")
        mode = VoxelDownsizeFilter::Mode::Center;
    else if (s == "first")
        mode = VoxelDownsizeFilter::Mode::First;
    else
        throw pdal_error("filters.voxeldownsize: Invalid 'mode' option '" +
            s + "'. " "Valid options are 'center' and 'first'");
    return in;
}


std::ostream& operator<<(std::ostream& out,
    const VoxelDownsizeFilter::Mode& mode)
{
    switch (mode)
    {
    case VoxelDownsizeFilter::Mode::Center:
        out << "center";
        break;
    case VoxelDownsizeFilter::Mode::First:
        out << "first";
        break;
    }
    return out;
}


VoxelDownsizeFilter::VoxelDownsizeFilter()
{}


std::string VoxelDownsizeFilter::getName() const
{
    return s_info.name;
}

void VoxelDownsizeFilter::addArgs(ProgramArgs& args)
{
    args.add("cell", "Cell size", m_cell, 0.001);
    args.add("mode", "Method for downsizing : center / first",
        m_mode, Mode::Center);
}


void VoxelDownsizeFilter::ready(PointTableRef)
{ m_populatedVoxels.clear(); }



PointViewSet VoxelDownsizeFilter::run(PointViewPtr view)
{
    PointViewPtr output = view->makeNew();
    PointRef point(*view);
    for (PointId id = 0; id < view->size(); ++id)
    {
        point.setPointId(id);
        if (voxelize(point))
            output->appendPoint(*view, id);
    }

    PointViewSet viewSet;
    viewSet.insert(output);
    return viewSet;
}

bool VoxelDownsizeFilter::find(int gx, int gy, int gz)
{
    //m_pool->await();
    auto itr = m_populatedVoxels.find(std::make_tuple(gx, gy, gz));
    if (itr == m_populatedVoxels.end())
    {
        std::string val;
        leveldb::Status s = m_ldb->Get(
                                leveldb::ReadOptions(),
                                std::to_string(gx) + std::to_string(gy) + std::to_string(gz), &val);
        return (s.ok() && !val.empty());
    }
    return true;
}

bool VoxelDownsizeFilter::insert(int gx, int gy, int gz)
{
    if (m_populatedVoxels.size() > m_batchSize)
    {
        std::set<std::tuple<int, int, int>> tempMap = m_populatedVoxels;
        leveldb::WriteBatch batch;
        for (auto itr=tempMap.begin(); itr!=tempMap.end(); ++itr)
        {
            auto t=*itr;
            auto val = std::to_string(std::get<0>(t)) +
                       std::to_string(std::get<1>(t)) +
                       std::to_string(std::get<2>(t));
            batch.Put(val,val);
        }
        auto res = m_ldb->Write(leveldb::WriteOptions(), &batch).ok();
        assert(res);
        m_populatedVoxels.clear();
    }
    return m_populatedVoxels.insert(std::make_tuple(gx, gy, gz)).second;
}


bool VoxelDownsizeFilter::voxelize(PointRef& point)
{
    /*
     * Calculate the voxel coordinates for the incoming point.
     * gx, gy, gz will be the global coordinates from (0, 0, 0).
     */
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);
    if (m_populatedVoxels.empty())
    {
        m_originX = x - (m_cell / 2);
        m_originY = y - (m_cell / 2);
        m_originZ = z - (m_cell / 2);
    }

    // Offset by origin.
    x -= m_originX;
    y -= m_originY;
    z -= m_originZ;

    Voxel v = std::make_tuple((int)(std::floor(x / m_cell)),
        (int)(std::floor(y / m_cell)), (int)(std::floor(z / m_cell)));

    auto inserted = m_populatedVoxels.insert(v).second;
    if ((m_mode == Mode::Center) && inserted)
    {
        point.setField(Dimension::Id::X,
            (std::get<0>(v) + 0.5) * m_cell + m_originX);
        point.setField(Dimension::Id::Y,
            (std::get<1>(v) + 0.5) * m_cell + m_originY);
        point.setField(Dimension::Id::Z,
            (std::get<2>(v) + 0.5) * m_cell + m_originZ);

    }
    return false;
}

bool VoxelDownsizeFilter::processOne(PointRef& point)
{
    return voxelize(point);
}

void VoxelDownsizeFilter::done(PointTableRef) {
    delete m_ldb;
}

} // namespace pdal
