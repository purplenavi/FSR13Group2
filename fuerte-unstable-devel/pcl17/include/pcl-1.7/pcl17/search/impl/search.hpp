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
 */

#ifndef PCL17_SEARCH_SEARCH_IMPL_HPP_
#define PCL17_SEARCH_SEARCH_IMPL_HPP_

#include <pcl17/search/search.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl17::search::Search<PointT>::Search (const std::string& name, bool sorted)
  : input_ () 
  , indices_ ()
  , sorted_results_ (sorted)
  , name_ (name)
{
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> const std::string& 
pcl17::search::Search<PointT>::getName () const
{
  return (name_);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl17::search::Search<PointT>::setSortedResults (bool sorted)
{
  sorted_results_ = sorted;
}
 
///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl17::search::Search<PointT>::setInputCloud (
    const PointCloudConstPtr& cloud, const IndicesConstPtr &indices)
{
  input_ = cloud;
  indices_ = indices;
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl17::search::Search<PointT>::nearestKSearch (
    const PointCloud &cloud, int index, int k,
    std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) const
{
  assert (index >= 0 && index < static_cast<int> (cloud.points.size ()) && "Out-of-bounds error in nearestKSearch!");
  return (nearestKSearch (cloud.points[index], k, k_indices, k_sqr_distances));
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl17::search::Search<PointT>::nearestKSearch (
    int index, int k, 
    std::vector<int> &k_indices, 
    std::vector<float> &k_sqr_distances) const
{
  if (indices_ == NULL)
  {
    assert (index >= 0 && index < static_cast<int> (input_->points.size ()) && "Out-of-bounds error in nearestKSearch!");
    return (nearestKSearch (input_->points[index], k, k_indices, k_sqr_distances));
  }
  else
  {
    assert (index >= 0 && index < static_cast<int> (indices_->size ()) && "Out-of-bounds error in nearestKSearch!");
    if (index >= static_cast<int> (indices_->size ()) || index < 0)
      return (0);
    return (nearestKSearch (input_->points[(*indices_)[index]], k, k_indices, k_sqr_distances));
  }
}
 
///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl17::search::Search<PointT>::nearestKSearch (
    const PointCloud& cloud, const std::vector<int>& indices, 
    int k, std::vector< std::vector<int> >& k_indices,
    std::vector< std::vector<float> >& k_sqr_distances) const
{
  if (indices.empty ())
  {
    k_indices.resize (cloud.size ());
    k_sqr_distances.resize (cloud.size ());
    for (size_t i = 0; i < cloud.size (); i++)
      nearestKSearch (cloud, static_cast<int> (i), k, k_indices[i], k_sqr_distances[i]);
  }
  else
  {
    k_indices.resize (indices.size ());
    k_sqr_distances.resize (indices.size ());
    for (size_t i = 0; i < indices.size (); i++)
      nearestKSearch (cloud, indices[i], k, k_indices[i], k_sqr_distances[i]);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl17::search::Search<PointT>::radiusSearch (
    const PointCloud &cloud, int index, double radius,
    std::vector<int> &k_indices, std::vector<float> &k_sqr_distances,
    unsigned int max_nn) const
{
  assert (index >= 0 && index < static_cast<int> (cloud.points.size ()) && "Out-of-bounds error in radiusSearch!");
  return (radiusSearch(cloud.points[index], radius, k_indices, k_sqr_distances, max_nn));
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl17::search::Search<PointT>::radiusSearch (
    int index, double radius, std::vector<int> &k_indices,
    std::vector<float> &k_sqr_distances, unsigned int max_nn ) const
{
  if (indices_ == NULL)
  {
    assert (index >= 0 && index < static_cast<int> (input_->points.size ()) && "Out-of-bounds error in radiusSearch!");
    return (radiusSearch (input_->points[index], radius, k_indices, k_sqr_distances, max_nn));
  }
  else
  {
    assert (index >= 0 && index < static_cast<int> (indices_->size ()) && "Out-of-bounds error in radiusSearch!");
    return (radiusSearch (input_->points[(*indices_)[index]], radius, k_indices, k_sqr_distances, max_nn));
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl17::search::Search<PointT>::radiusSearch (
    const PointCloud& cloud,
    const std::vector<int>& indices,
    double radius,
    std::vector< std::vector<int> >& k_indices,
    std::vector< std::vector<float> > &k_sqr_distances,
    unsigned int max_nn) const
{
  if (indices.empty ())
  {
    k_indices.resize (cloud.size ());
    k_sqr_distances.resize (cloud.size ());
    for (size_t i = 0; i < cloud.size (); i++)
      radiusSearch (cloud, static_cast<int> (i), radius,k_indices[i], k_sqr_distances[i], max_nn);
  }
  else
  {
    k_indices.resize (indices.size ());
    k_sqr_distances.resize (indices.size ());
    for (size_t i = 0; i < indices.size (); i++)
      radiusSearch (cloud,indices[i],radius,k_indices[i],k_sqr_distances[i], max_nn);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl17::search::Search<PointT>::sortResults (
    std::vector<int>& indices, std::vector<float>& distances) const
{
  std::vector<int> order (indices.size ());
  for (size_t idx = 0; idx < order.size (); ++idx)
    order [idx] = static_cast<int> (idx);

  Compare compare (distances);
  sort (order.begin (), order.end (), compare);

  std::vector<int> sorted (indices.size ());
  for (size_t idx = 0; idx < order.size (); ++idx)
    sorted [idx] = indices[order [idx]];

  indices = sorted;

  // sort  the according distances.
  sort (distances.begin (), distances.end ());
}

#define PCL17_INSTANTIATE_Search(T) template class PCL17_EXPORTS pcl17::search::Search<T>;

#endif  //#ifndef _PCL17_SEARCH_SEARCH_IMPL_HPP_

