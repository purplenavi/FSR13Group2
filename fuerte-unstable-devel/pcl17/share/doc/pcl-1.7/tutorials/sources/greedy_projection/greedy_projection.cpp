#include <pcl17/point_types.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/kdtree/kdtree_flann.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/surface/gp3.h>

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>);
  sensor_msgs::PointCloud2 cloud_blob;
  pcl17::io::loadPCDFile ("bun0.pcd", cloud_blob);
  pcl17::fromROSMsg (cloud_blob, *cloud);
  //* the data should be available in cloud

  // Normal estimation*
  pcl17::NormalEstimation<pcl17::PointXYZ, pcl17::Normal> n;
  pcl17::PointCloud<pcl17::Normal>::Ptr normals (new pcl17::PointCloud<pcl17::Normal>);
  pcl17::search::KdTree<pcl17::PointXYZ>::Ptr tree (new pcl17::search::KdTree<pcl17::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl17::PointCloud<pcl17::PointNormal>::Ptr cloud_with_normals (new pcl17::PointCloud<pcl17::PointNormal>);
  pcl17::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl17::search::KdTree<pcl17::PointNormal>::Ptr tree2 (new pcl17::search::KdTree<pcl17::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl17::GreedyProjectionTriangulation<pcl17::PointNormal> gp3;
  pcl17::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Finish
  return (0);
}
