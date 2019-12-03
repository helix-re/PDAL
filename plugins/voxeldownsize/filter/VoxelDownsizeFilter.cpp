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
#include <algorithm>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.voxeldownsize",
    "First Entry Voxel Filter",
    "http://pdal.io/stages/filters.voxeldownsize.html"
};

CREATE_SHARED_STAGE(VoxelDownsizeFilter, s_info)

VoxelDownsizeFilter::VoxelDownsizeFilter()
{}


std::string VoxelDownsizeFilter::getName() const
{
    return s_info.name;
}

void VoxelDownsizeFilter::addArgs(ProgramArgs& args)
{
    args.add("cell", "Cell size", m_cell, 0.001);
    args.add("mode", "Method for downsizing : voxelcenter / firstinvoxel",
             m_mode, "voxelcenter");
}


void VoxelDownsizeFilter::ready(PointTableRef)
{
    m_pivotVoxelInitialized = false;
    leveldb::Options ldbOptions;
    ldbOptions.create_if_missing = true;
    //ldbOptions.compression = leveldb::kNoCompression;
    ldbOptions.block_cache = leveldb::NewLRUCache(1 * 1024 * 1024 * 1024);
    std::time_t time;
    std::time(&time);
    log()->get(LogLevel::Debug)<<time<<std::endl;
    std::string dbPath = arbiter::getTempPath() + "/" + std::to_string(time);
    leveldb::Status status = leveldb::DB::Open(ldbOptions, dbPath, (leveldb::DB**)&m_ldb);
    assert(status.ok());

    m_pool.reset(new Pool(4, 10));
    m_batchSize = 10000000;
    m_doLevelDBSearch = false;

}
void VoxelDownsizeFilter::prepared(PointTableRef)
{
    if (m_mode.compare("voxelcenter")!=0 && m_mode.compare("firstinvoxel")!=0)
        throw pdal_error("Invalid Downsizing mode");

    m_isFirstInVoxelMode = (m_mode.compare("firstinvoxel") == 0);
}


PointViewSet VoxelDownsizeFilter::run(PointViewPtr view)
{
    PointViewPtr output = view->makeNew();
    for (PointId id = 0; id < view->size(); ++id)
    {
        if (voxelize(view->point(id)))
        {
            output->appendPoint(*view, id);
        }
    }

    PointViewSet viewSet;
    viewSet.insert(output);
    return viewSet;
}

bool VoxelDownsizeFilter::find(int gx, int gy, int gz)
{
    auto itr = m_populatedVoxels.find(std::make_tuple(gx, gy, gz));
    if (itr == m_populatedVoxels.end())
    {
        if (m_doLevelDBSearch)
        {
            std::unique_lock<std::mutex> lk(m_mutex);
            m_cvLock.wait(lk,[this]()
            {
                return !m_syncing;
            });
            std::string val;
            leveldb::Status s = m_ldb->Get(
                                    leveldb::ReadOptions(),
                                    std::to_string(gx) + std::to_string(gy) + std::to_string(gz), &val);
            return (s.ok() && !val.empty());
        }
        return false;
    }
    return true;
}

bool VoxelDownsizeFilter::insert(int gx, int gy, int gz)
{
    if (m_populatedVoxels.size() > m_batchSize)
    {
        m_doLevelDBSearch = true;
        m_syncSet.clear();
        std::swap(m_syncSet, m_populatedVoxels);
        m_pool->add([this]()
        {
            m_syncing = true;
            std::unique_lock<std::mutex> lock(m_mutex);

            ///////////////////////////////////////////////////////////
            Pool innerPool(8,100);
            size_t chunkSize = m_batchSize / 100;
            while (m_syncSet.size()>0){
                auto itr = m_syncSet.begin();
                std::advance(itr, (std::min)(chunkSize, m_syncSet.size()));
                std::set<std::tuple<int, int, int>> tempSet(m_syncSet.begin(), itr);
                m_syncSet.erase(m_syncSet.begin(), itr);
                auto db = m_ldb.get();
                innerPool.add([db, tempSet](){
                    leveldb::WriteBatch batch;
                    for (auto itr = tempSet.begin(); itr != tempSet.end(); ++itr)
                    {
                        auto t = *itr;
                        auto key = std::to_string(std::get<0>(t)) +
                                   std::to_string(std::get<1>(t)) +
                                   std::to_string(std::get<2>(t));
                        batch.Put(key, "1");
                    }
                    auto res = db->Write(leveldb::WriteOptions(), &batch).ok();
                    assert(res);
                });
            }
            innerPool.join();

            //////////////////////////////////////////////////////////

//            leveldb::WriteBatch batch;
//            for (auto itr = syncSet.begin(); itr != syncSet.end(); ++itr)
//            {
//                auto t = *itr;
//                auto key = std::to_string(std::get<0>(t)) +
//                           std::to_string(std::get<1>(t)) +
//                           std::to_string(std::get<2>(t));
//                batch.Put(key, "1");
//            }
//            auto res = m_ldb->Write(leveldb::WriteOptions(), &batch).ok();
//            assert(res);

            m_syncing = false;
            lock.unlock();
            m_cvLock.notify_one();
        });
        m_pool->go();
    }
    return m_populatedVoxels.insert(std::make_tuple(gx, gy, gz)).second;
}


bool VoxelDownsizeFilter::voxelize(PointRef point)
{
    /*
     * Calculate the voxel coordinates for the incoming point.
     * gx, gy, gz will be the global coordinates from (0, 0, 0).
     */
    int gx = std::floor(point.getFieldAs<double>(Dimension::Id::X) / m_cell);
    int gy = std::floor(point.getFieldAs<double>(Dimension::Id::Y) / m_cell);
    int gz = std::floor(point.getFieldAs<double>(Dimension::Id::Z) / m_cell);

    if (!m_pivotVoxelInitialized)
    {
        /*
         * Save global coordinates of first incoming point's voxel.
         * This will act as a Pivot for calculation of local coordinates of the
         * voxels.
         */
        m_pivotVoxel[0] = gx; // X Coordinate of an Pivot voxel
        m_pivotVoxel[1] = gy; // Y Coordinate of an Pivot voxel
        m_pivotVoxel[2] = gz; // Z Coordinate of an Pivot voxel
        m_pivotVoxelInitialized = true;
    }

    /*
     * Calculate the local voxel coordinates for incoming point, Using the
     * Pivot voxel.
     */
    auto kx = gx - m_pivotVoxel[0], ky = gy - m_pivotVoxel[1],
         kz = gz - m_pivotVoxel[2];
    if (!find(kx, ky, kz))
    {
        if (!m_isFirstInVoxelMode)
        {
            point.setField<double>(Dimension::Id::X, (gx + 0.5) * m_cell);
            point.setField<double>(Dimension::Id::Y, (gy + 0.5) * m_cell);
            point.setField<double>(Dimension::Id::Z, (gz + 0.5) * m_cell);
        }
        return insert(kx, ky, kz);
    }
    return false;
}

bool VoxelDownsizeFilter::processOne(PointRef& point)
{
    static point_count_t c = 0;
    c++;
    if(c%1000000 ==0){
        log()->get(LogLevel::Debug) << "Wrote " << c << " points"<<std::endl;
    }
    return voxelize(point);
}

void VoxelDownsizeFilter::done(PointTableRef)
{
    delete m_pool.release();
    delete m_ldb.release();

    std::time_t time;
    std::time(&time);
    log()->get(LogLevel::Debug)<<time<<std::endl;
}

} // namespace pdal
