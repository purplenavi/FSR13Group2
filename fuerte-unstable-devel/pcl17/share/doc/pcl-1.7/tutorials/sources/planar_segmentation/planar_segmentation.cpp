#include <iostream>
#include <pcl17/ModelCoefficients.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>

int
 main (int argc, char** argv)
{
  pcl17::PointCloud<pcl17::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width  = 15;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  // Generate the data
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1.0;
  }

  // Set a few outliers
  cloud.points[0].z = 2.0;
  cloud.points[3].z = -2.0;
  cloud.points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " 
                        << cloud.points[i].y << " " 
                        << cloud.points[i].z << std::endl;

  pcl17::ModelCoefficients::Ptr coefficients (new pcl17::ModelCoefficients);
  pcl17::PointIndices::Ptr inliers (new pcl17::PointIndices);
  // Create the segmentation object
  pcl17::SACSegmentation<pcl17::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl17::SACMODEL_PLANE);
  seg.setMethodType (pcl17::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL17_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
                                               << cloud.points[inliers->indices[i]].y << " "
                                               << cloud.points[inliers->indices[i]].z << std::endl;

  return (0);
}
