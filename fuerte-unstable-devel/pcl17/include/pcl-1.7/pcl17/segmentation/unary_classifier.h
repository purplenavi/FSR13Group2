/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#ifndef PCL17_UNARY_CLASSIFIER_H_
#define PCL17_UNARY_CLASSIFIER_H_

#include <pcl17/point_cloud.h>
#include <pcl17/point_types.h>

#include <pcl17/features/fpfh.h>
#include <pcl17/features/normal_3d.h>

#include <pcl17/filters/filter_indices.h>
#include <pcl17/segmentation/extract_clusters.h>

#include <pcl17/ml/kmeans.h>

namespace pcl17
{
  /** \brief
    * 
    */
  template <typename PointT>
  class PCL17_EXPORTS UnaryClassifier
  {
    public:

      /** \brief Constructor that sets default values for member variables. */
      UnaryClassifier ();

      /** \brief This destructor destroys the cloud...
        * 
        */
      ~UnaryClassifier ();

      /** \brief This method sets the input cloud.
        * \param[in] input_cloud input point cloud
        */
      void
      setInputCloud (typename pcl17::PointCloud<PointT>::Ptr input_cloud);

      void 
      train (pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr &output);
      
      void
      trainWithLabel (std::vector<pcl17::PointCloud<pcl17::FPFHSignature33>, Eigen::aligned_allocator<pcl17::PointCloud<pcl17::FPFHSignature33> > > &output);

      void
      segment (pcl17::PointCloud<pcl17::PointXYZRGBL>::Ptr &out);

      void
      queryFeatureDistances (std::vector<pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr> &trained_features,
                             pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr query_features,
                             std::vector<int> &indi,
                             std::vector<float> &dist);

      void
      assignLabels (std::vector<int> &indi,
                    std::vector<float> &dist,
                    int n_feature_means,
                    float feature_threshold,
                    pcl17::PointCloud<pcl17::PointXYZRGBL>::Ptr out);

      void
      setClusterSize (unsigned int k){cluster_size_ = k;};
      
      void
      setNormalRadiusSearch (float param){normal_radius_search_ = param;};
      
      void
      setFPFHRadiusSearch (float param){fpfh_radius_search_ = param;};
      
      void
      setLabelField (bool l){label_field_ = l;};

      void
      setTrainedFeatures (std::vector<pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr> &features){trained_features_ = features;};

      void
      setFeatureThreshold (float threshold){feature_threshold_ = threshold;};

    protected:

      void
      convertCloud (typename pcl17::PointCloud<PointT>::Ptr in,
                    pcl17::PointCloud<pcl17::PointXYZ>::Ptr out);

      void
      convertCloud (typename pcl17::PointCloud<PointT>::Ptr in,
                    pcl17::PointCloud<pcl17::PointXYZRGBL>::Ptr out);

      void
      findClusters (typename pcl17::PointCloud<PointT>::Ptr in,
                    std::vector<int> &cluster_numbers);

      void
      getCloudWithLabel (typename pcl17::PointCloud<PointT>::Ptr in,
                         pcl17::PointCloud<pcl17::PointXYZ>::Ptr out,
                         int label_num);

      void
      computeFPFH (pcl17::PointCloud<pcl17::PointXYZ>::Ptr in,
                   pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr out,
                   float normal_radius_search,
                   float fpfh_radius_search);
      
      void
      kmeansClustering (pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr in,
                        pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr out,
                        int k);



      /** \brief Contains the input cloud */
      typename pcl17::PointCloud<PointT>::Ptr input_cloud_;
      
      bool label_field_;
      
      unsigned int cluster_size_;

      float normal_radius_search_;
      float fpfh_radius_search_;
      float feature_threshold_;
      
      
      std::vector<pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr> trained_features_;

      /** \brief Contains normals of the points that will be segmented. */
      //typename pcl17::PointCloud<pcl17::Normal>::Ptr normals_;

      /** \brief Stores the cloud that will be segmented. */
      //typename pcl17::PointCloud<PointT>::Ptr cloud_for_segmentation_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
}

#ifdef PCL17_NO_PRECOMPILE
#include <pcl17/segmentation/impl/unary_classifier.hpp>
#endif

#endif
