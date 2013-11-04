#include <pcl17/ModelCoefficients.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/filters/passthrough.h>
#include <pcl17/filters/project_inliers.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/surface/convex_hull.h>

int
 main (int argc, char** argv)
{
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>), cloud_filtered (new pcl17::PointCloud<pcl17::PointXYZ>), cloud_projected (new pcl17::PointCloud<pcl17::PointXYZ>);
  pcl17::PCDReader reader;
  reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);

  // Build a filter to remove spurious NaNs
  pcl17::PassThrough<pcl17::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.1);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

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

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  // Project the model inliers
  pcl17::ProjectInliers<pcl17::PointXYZ> proj;
  proj.setModelType (pcl17::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setIndices (inliers);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Create a Convex Hull representation of the projected inliers
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_hull (new pcl17::PointCloud<pcl17::PointXYZ>);
  pcl17::ConvexHull<pcl17::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;

  pcl17::PCDWriter writer;
  writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

  return (0);
}
