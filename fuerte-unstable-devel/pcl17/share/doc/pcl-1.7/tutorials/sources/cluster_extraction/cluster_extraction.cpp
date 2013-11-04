#include <pcl17/ModelCoefficients.h>
#include <pcl17/point_types.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/filters/extract_indices.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/kdtree/kdtree.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/segmentation/extract_clusters.h>


int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl17::PCDReader reader;
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>), cloud_f (new pcl17::PointCloud<pcl17::PointXYZ>);
  reader.read ("table_scene_lms400.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl17::VoxelGrid<pcl17::PointXYZ> vg;
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_filtered (new pcl17::PointCloud<pcl17::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl17::SACSegmentation<pcl17::PointXYZ> seg;
  pcl17::PointIndices::Ptr inliers (new pcl17::PointIndices);
  pcl17::ModelCoefficients::Ptr coefficients (new pcl17::ModelCoefficients);
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_plane (new pcl17::PointCloud<pcl17::PointXYZ> ());
  pcl17::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl17::SACMODEL_PLANE);
  seg.setMethodType (pcl17::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl17::ExtractIndices<pcl17::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl17::search::KdTree<pcl17::PointXYZ>::Ptr tree (new pcl17::search::KdTree<pcl17::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl17::PointIndices> cluster_indices;
  pcl17::EuclideanClusterExtraction<pcl17::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl17::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_cluster (new pcl17::PointCloud<pcl17::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl17::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}
