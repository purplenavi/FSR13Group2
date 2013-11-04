/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#ifndef PCL17_KEYPOINTS_AGAST_KEYPOINT_2D_IMPL_H_
#define PCL17_KEYPOINTS_AGAST_KEYPOINT_2D_IMPL_H_

#include <pcl17/common/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> bool
pcl17::AgastKeypoint2DBase<PointInT, PointOutT, IntensityT>::initCompute ()
{
  if (!pcl17::Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL17_ERROR ("[pcl17::%s::initCompute] init failed.!\n", name_.c_str ());
    return (false);
  }

  if (!input_->isOrganized ())
  {    
    PCL17_ERROR ("[pcl17::%s::initCompute] %s doesn't support non organized clouds!\n", name_.c_str ());
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl17::AgastKeypoint2D<PointInT, PointOutT>::detectKeypoints (PointCloudOut &output)
{
  // image size
  const size_t width = input_->width;
  const size_t height = input_->height;

  // destination for intensity data; will be forwarded to AGAST
  std::vector<unsigned char> image_data (width*height);

  for (size_t i = 0; i < image_data.size (); ++i)
    image_data[i] = static_cast<unsigned char> (intensity_ ((*input_)[i]));

  if (!detector_)
    detector_.reset (new pcl17::keypoints::agast::AgastDetector7_12s (width, height, threshold_, bmax_));

  detector_->setMaxKeypoints (nr_max_keypoints_);

  if (apply_non_max_suppression_)
  {
    pcl17::PointCloud<pcl17::PointUV> tmp_cloud;
    detector_->detectKeypoints (image_data, tmp_cloud);

    pcl17::keypoints::internal::AgastApplyNonMaxSuppresion<PointOutT> anms (
        image_data, tmp_cloud, detector_, output);
  }
  else
  {
    pcl17::keypoints::internal::AgastDetector<PointOutT> dec (
        image_data, detector_, output);
  }

  // we do not change the denseness
  output.is_dense = true;
}


#define AgastKeypoint2D(T,I) template class PCL17_EXPORTS pcl17::AgastKeypoint2D<T,I>;
#endif 
