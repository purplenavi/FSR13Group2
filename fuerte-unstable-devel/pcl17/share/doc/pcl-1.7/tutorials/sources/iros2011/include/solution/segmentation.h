#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "typedefs.h"

#include <pcl17/ModelCoefficients.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/filters/extract_indices.h>
#include <pcl17/segmentation/extract_clusters.h>


/* Use SACSegmentation to find the dominant plane in the scene
 * Inputs:
 *   input 
 *     The input point cloud
 *   max_iterations 
 *     The maximum number of RANSAC iterations to run
 *   distance_threshold 
 *     The inlier/outlier threshold.  Points within this distance
 *     from the hypothesized plane are scored as inliers.
 * Return: A pointer to the ModelCoefficients (i.e., the 4 coefficients of the plane, 
 *         represented in c0*x + c1*y + c2*z + c3 = 0 form)
 */
pcl17::ModelCoefficients::Ptr
fitPlane (const PointCloudPtr & input, float distance_threshold, float max_iterations)
{
  // Intialize the SACSegmentation object
  pcl17::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl17::SACMODEL_PLANE);
  seg.setMethodType (pcl17::SAC_RANSAC);
  seg.setDistanceThreshold (distance_threshold);
  seg.setMaxIterations (max_iterations);

  seg.setInputCloud (input);
  pcl17::ModelCoefficients::Ptr coefficients (new pcl17::ModelCoefficients ());
  pcl17::PointIndices::Ptr inliers (new pcl17::PointIndices ());
  seg.segment (*inliers, *coefficients);  

  return (coefficients);
}

/* Use SACSegmentation and an ExtractIndices filter to find the dominant plane and subtract it
 * Inputs:
 *   input 
 *     The input point cloud
 *   max_iterations 
 *     The maximum number of RANSAC iterations to run
 *   distance_threshold 
 *     The inlier/outlier threshold.  Points within this distance
 *     from the hypothesized plane are scored as inliers.
 * Return: A pointer to a new point cloud which contains only the non-plane points
 */
PointCloudPtr
findAndSubtractPlane (const PointCloudPtr & input, float distance_threshold, float max_iterations)
{
  // Find the dominant plane
  pcl17::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients (false);
  seg.setModelType (pcl17::SACMODEL_PLANE);
  seg.setMethodType (pcl17::SAC_RANSAC);
  seg.setDistanceThreshold (distance_threshold);
  seg.setMaxIterations (max_iterations);
  seg.setInputCloud (input);
  pcl17::ModelCoefficients::Ptr coefficients (new pcl17::ModelCoefficients ());
  pcl17::PointIndices::Ptr inliers (new pcl17::PointIndices ());
  seg.segment (*inliers, *coefficients);  

  // Extract the inliers
  pcl17::ExtractIndices<PointT> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (true);
  PointCloudPtr output (new PointCloud);
  extract.filter (*output);

  return (output);
}

/* Use EuclidieanClusterExtraction to group a cloud into contiguous clusters
 * Inputs:
 *   input
 *     The input point cloud
 *   cluster_tolerance
 *     The maximum distance between neighboring points in a cluster
 *   min/max_cluster_size
 *     The minimum and maximum allowable cluster sizes
 * Return (by reference): a vector of PointIndices containing the points indices in each cluster
 */
void
clusterObjects (const PointCloudPtr & input, 
                float cluster_tolerance, int min_cluster_size, int max_cluster_size,
                std::vector<pcl17::PointIndices> & cluster_indices_out)
{  
  pcl17::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);

  ec.setInputCloud (input);
  ec.extract (cluster_indices_out);
}

#endif
