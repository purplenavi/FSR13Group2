/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id: point_types.hpp 6415 2012-07-16 20:11:47Z rusu $
 *
 */

#ifndef PCL17_COMMON_POINT_TESTS_H_
#define PCL17_COMMON_POINT_TESTS_H_

#ifdef _MSC_VER
#include <Eigen/src/StlSupport/details.h>
#endif

namespace pcl17
{
  /** Tests if the 3D components of a point are all finite
    * param[in] pt point to be tested
    */
  template <typename PointT> inline bool
  isFinite (const PointT &pt)
  {
    return (pcl_isfinite (pt.x) && pcl_isfinite (pt.y) && pcl_isfinite (pt.z));
  }

#ifdef _MSC_VER
  template <typename PointT> inline bool
  isFinite (const Eigen::internal::workaround_msvc_stl_support<PointT> &pt)
  {
    return isFinite<PointT> (static_cast<const PointT&> (pt));
  }
#endif

  template<> inline bool isFinite<pcl17::RGB> (const pcl17::RGB&) { return (true); }
  template<> inline bool isFinite<pcl17::Label> (const pcl17::Label&) { return (true); }
  template<> inline bool isFinite<pcl17::Axis> (const pcl17::Axis&) { return (true); }
  template<> inline bool isFinite<pcl17::MomentInvariants> (const pcl17::MomentInvariants&) { return (true); }
  template<> inline bool isFinite<pcl17::PrincipalRadiiRSD> (const pcl17::PrincipalRadiiRSD&) { return (true); }
  template<> inline bool isFinite<pcl17::Boundary> (const pcl17::Boundary&) { return (true); }
  template<> inline bool isFinite<pcl17::PrincipalCurvatures> (const pcl17::PrincipalCurvatures&) { return (true); }
  template<> inline bool isFinite<pcl17::SHOT352> (const pcl17::SHOT352&) { return (true); }
  template<> inline bool isFinite<pcl17::SHOT1344> (const pcl17::SHOT1344&) { return (true); }
  template<> inline bool isFinite<pcl17::ReferenceFrame> (const pcl17::ReferenceFrame&) { return (true); }
  template<> inline bool isFinite<pcl17::ShapeContext1980> (const pcl17::ShapeContext1980&) { return (true); }
  template<> inline bool isFinite<pcl17::PFHSignature125> (const pcl17::PFHSignature125&) { return (true); }
  template<> inline bool isFinite<pcl17::PFHRGBSignature250> (const pcl17::PFHRGBSignature250&) { return (true); }
  template<> inline bool isFinite<pcl17::PPFSignature> (const pcl17::PPFSignature&) { return (true); }
  template<> inline bool isFinite<pcl17::PPFRGBSignature> (const pcl17::PPFRGBSignature&) { return (true); }
  template<> inline bool isFinite<pcl17::NormalBasedSignature12> (const pcl17::NormalBasedSignature12&) { return (true); }
  template<> inline bool isFinite<pcl17::FPFHSignature33> (const pcl17::FPFHSignature33&) { return (true); }
  template<> inline bool isFinite<pcl17::VFHSignature308> (const pcl17::VFHSignature308&) { return (true); }
  template<> inline bool isFinite<pcl17::ESFSignature640> (const pcl17::ESFSignature640&) { return (true); }
  template<> inline bool isFinite<pcl17::IntensityGradient> (const pcl17::IntensityGradient&) { return (true); }
  template<> inline bool isFinite<pcl17::BRISKSignature512> (const pcl17::BRISKSignature512&) { return (true); }

  // specification for pcl17::PointXY
  template <> inline bool
  isFinite<pcl17::PointXY> (const pcl17::PointXY &p)
  {
    return (pcl_isfinite (p.x) && pcl_isfinite (p.y));
  }

  // specification for pcl17::BorderDescription
  template <> inline bool
  isFinite<pcl17::BorderDescription> (const pcl17::BorderDescription &p)
  {
    return (pcl_isfinite (p.x) && pcl_isfinite (p.y));
  }

  // specification for pcl17::Normal
  template <> inline bool
  isFinite<pcl17::Normal> (const pcl17::Normal &n)
  {
    return (pcl_isfinite (n.normal_x) && pcl_isfinite (n.normal_y) && pcl_isfinite (n.normal_z));
  }
}

#endif    // PCL17_COMMON_POINT_TESTS_H_

