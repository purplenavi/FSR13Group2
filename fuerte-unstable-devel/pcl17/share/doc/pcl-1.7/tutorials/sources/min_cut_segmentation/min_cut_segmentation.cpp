#include <iostream>
#include <vector>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>
#include <pcl17/visualization/cloud_viewer.h>
#include <pcl17/filters/passthrough.h>
#include <pcl17/segmentation/min_cut_segmentation.h>

int main (int argc, char** argv)
{
  pcl17::PointCloud <pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud <pcl17::PointXYZ>);
  if ( pcl17::io::loadPCDFile <pcl17::PointXYZ> ("min_cut_segmentation_tutorial.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl17::IndicesPtr indices (new std::vector <int>);
  pcl17::PassThrough<pcl17::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl17::MinCutSegmentation<pcl17::PointXYZ> seg;
  seg.setInputCloud (cloud);
  seg.setIndices (indices);

  pcl17::PointCloud<pcl17::PointXYZ>::Ptr foreground_points(new pcl17::PointCloud<pcl17::PointXYZ> ());
  pcl17::PointXYZ point;
  point.x = 68.97;
  point.y = -18.55;
  point.z = 0.57;
  foreground_points->points.push_back(point);
  seg.setForegroundPoints (foreground_points);

  seg.setSigma (0.25);
  seg.setRadius (3.0433856);
  seg.setNumberOfNeighbours (14);
  seg.setSourceWeight (0.8);

  std::vector <pcl17::PointIndices> clusters;
  seg.extract (clusters);

  std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

  pcl17::PointCloud <pcl17::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
  pcl17::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}