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

#pragma once

#include <E57Format.h>
#include <pdal/Dimension.hpp>
#include <pdal/PointRef.hpp>
namespace pdal
{
namespace e57plugin
{
/// converts a E57 string to the corresponding pdal dimension.
/// returns pdal::Dimension::Id::Unknown in case the dimension is
/// not recognised
PDAL_DLL inline Dimension::Id e57ToPdal(const std::string& e57Dimension);

/// Applies rotation and translation on point.
PDAL_DLL inline void transformPoint(PointRef& pt, const double rotation[3][3],
                                    const double m_translation[3]);

/// Returns a list of PDAL supported E57 dimensions.
PDAL_DLL inline std::vector<std::string> supportedE57Types();

/// Decodes pose for the scan (if any). Pose is nothing but a transformation to
/// be applied on a scan. If pose is present, this will return true and modify
/// rotation and translation matrices accordingly otherwise returns false and
/// donot modify rotation and translation matrices. If pose is present but no
/// rotation then rotation matrix will be a Identity matrix. If pose is present
/// but no translation then translation matrix will be [0, 0, 0].
PDAL_DLL bool getPose(const e57::StructureNode scan, double (&rotation)[3][3],
                      double (&translation)[3]);

/// Returns total number of points in data3D.
/// Where data3D is a "/data3D" node from E57 hierarchy.
PDAL_DLL point_count_t numPoints(const e57::VectorNode data3D);

/// Tries to find the limit of a dimension in the e57 node headers
/// if found, Fill minmax with minimum limit and maximum limit and return true
/// otherwise returns false. if not found, minmax will be a pair of {0.0, 0.0}
/// (double values).
PDAL_DLL bool getLimits(const e57::StructureNode& scan,
                        const std::string& fieldName,
                        std::pair<double, double>& minmax);
} // namespace e57plugin

} // namespace pdal