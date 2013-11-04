#include <pcl17/point_types.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/console/time.h>

#include <pcl17/filters/voxel_grid.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/segmentation/conditional_euclidean_clustering.h>

typedef pcl17::PointXYZI PointTypeIO;
typedef pcl17::PointXYZINormal PointTypeFull;

bool
enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  else
    return (false);
}

bool
enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
  if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  if (fabs (point_a_normal.dot (point_b_normal)) < 0.05)
    return (true);
  return (false);
}

bool
customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
  if (squared_distance < 10000)
  {
    if (fabs (point_a.intensity - point_b.intensity) < 8.0f)
      return (true);
    if (fabs (point_a_normal.dot (point_b_normal)) < 0.06)
      return (true);
  }
  else
  {
    if (fabs (point_a.intensity - point_b.intensity) < 3.0f)
      return (true);
  }
  return (false);
}

int
main (int argc, char** argv)
{
  // Data containers used
  pcl17::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl17::PointCloud<PointTypeIO>), cloud_out (new pcl17::PointCloud<PointTypeIO>);
  pcl17::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl17::PointCloud<PointTypeFull>);
  pcl17::IndicesClustersPtr clusters (new pcl17::IndicesClusters), small_clusters (new pcl17::IndicesClusters), large_clusters (new pcl17::IndicesClusters);
  pcl17::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl17::search::KdTree<PointTypeIO>);
  pcl17::console::TicToc tt;

  // Load the input point cloud
  std::cerr << "Loading...\n", tt.tic ();
  pcl17::io::loadPCDFile ("Statues_4.pcd", *cloud_in);
  std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->points.size () << " points\n";

  // Downsample the cloud using a Voxel Grid class
  std::cerr << "Downsampling...\n", tt.tic ();
  pcl17::VoxelGrid<PointTypeIO> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (80.0, 80.0, 80.0);
  vg.setDownsampleAllData (true);
  vg.filter (*cloud_out);
  std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_out->points.size () << " points\n";

  // Set up a Normal Estimation class and merge data in cloud_with_normals
  std::cerr << "Computing normals...\n", tt.tic ();
  pcl17::copyPointCloud (*cloud_out, *cloud_with_normals);
  pcl17::NormalEstimation<PointTypeIO, PointTypeFull> ne;
  ne.setInputCloud (cloud_out);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (300.0);
  ne.compute (*cloud_with_normals);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  // Set up a Conditional Euclidean Clustering class
  std::cerr << "Segmenting to clusters...\n", tt.tic ();
  pcl17::ConditionalEuclideanClustering<PointTypeFull> cec (true);
  cec.setInputCloud (cloud_with_normals);
  cec.setConditionFunction (&customRegionGrowing);
  cec.setClusterTolerance (500.0);
  cec.setMinClusterSize (cloud_with_normals->points.size () / 1000);
  cec.setMaxClusterSize (cloud_with_normals->points.size () / 5);
  cec.segment (*clusters);
  cec.getRemovedClusters (small_clusters, large_clusters);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  // Using the intensity channel for lazy visualization of the output
  for (int i = 0; i < small_clusters->size (); ++i)
    for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
      cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
  for (int i = 0; i < large_clusters->size (); ++i)
    for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
      cloud_out->points[(*large_clusters)[i].indices[j]].intensity = +10.0;
  for (int i = 0; i < clusters->size (); ++i)
  {
    int label = rand () % 8;
    for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
      cloud_out->points[(*clusters)[i].indices[j]].intensity = label;
  }

  // Save the output point cloud
  std::cerr << "Saving...\n", tt.tic ();
  pcl17::io::savePCDFile ("output.pcd", *cloud_out);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

  return (0);
}
