/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: point_cloud.h 4696 2012-02-23 06:12:55Z rusu $
 *
 */

#ifndef PCL17_TRACKING_TRACKING_H_
#define PCL17_TRACKING_TRACKING_H_

#include <pcl17/point_types.h>

#ifdef BUILD_Maintainer
#  if defined __GNUC__
#      pragma GCC system_header 
#  elif defined _MSC_VER
#    pragma warning(push, 1)
#  endif
#endif

namespace pcl17
{
  namespace tracking
  {
    /* state definition */
    struct ParticleXYZRPY;
    struct ParticleXYR;

    /* \brief return the value of normal distribution */
    PCL17_EXPORTS double
    sampleNormal (double mean, double sigma);
  }
}

#include <pcl17/tracking/impl/tracking.hpp>

// ==============================
// =====PCL17_POINT_CLOUD_REGISTER=====
// ==============================
PCL17_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl17::tracking::_ParticleXYZRPY,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
)
PCL17_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl17::tracking::ParticleXYZRPY, pcl17::tracking::_ParticleXYZRPY)


PCL17_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl17::tracking::_ParticleXYRPY,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
)
PCL17_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl17::tracking::ParticleXYRPY, pcl17::tracking::_ParticleXYRPY)


PCL17_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl17::tracking::_ParticleXYRP,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
)
PCL17_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl17::tracking::ParticleXYRP, pcl17::tracking::_ParticleXYRP)


PCL17_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl17::tracking::_ParticleXYR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
)
PCL17_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl17::tracking::ParticleXYR, pcl17::tracking::_ParticleXYR)

PCL17_POINT_CLOUD_REGISTER_POINT_STRUCT (pcl17::tracking::_ParticleXYZR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
)
PCL17_POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl17::tracking::ParticleXYZR, pcl17::tracking::_ParticleXYZR)

#ifdef BUILD_Maintainer
#  if defined _MSC_VER
#    pragma warning(pop)
#  endif
#endif

#ifdef PCL17_NO_PRECOMPILE
#include <pcl17/tracking/impl/tracking.hpp>
#endif

#endif