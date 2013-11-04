#include <iostream>
#include <vector>
#include <pcl17/point_types.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/search/search.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/visualization/cloud_viewer.h>
#include <pcl17/filters/passthrough.h>
#include <pcl17/segmentation/region_growing.h>

int
main (int argc, char** argv)
{
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>);
  if ( pcl17::io::loadPCDFile <pcl17::PointXYZ> ("region_growing_tutorial.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl17::search::Search<pcl17::PointXYZ>::Ptr tree = boost::shared_ptr<pcl17::search::Search<pcl17::PointXYZ> > (new pcl17::search::KdTree<pcl17::PointXYZ>);
  pcl17::PointCloud <pcl17::Normal>::Ptr normals (new pcl17::PointCloud <pcl17::Normal>);
  pcl17::NormalEstimation<pcl17::PointXYZ, pcl17::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl17::IndicesPtr indices (new std::vector <int>);
  pcl17::PassThrough<pcl17::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl17::RegionGrowing<pcl17::PointXYZ, pcl17::Normal> reg;
  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (10000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (7.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl17::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
  while (counter < 5 || counter > clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << std::endl;
    counter++;
  }

  pcl17::PointCloud <pcl17::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl17::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}
