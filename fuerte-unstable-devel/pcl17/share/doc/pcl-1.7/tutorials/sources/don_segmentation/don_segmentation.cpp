/**
 * @file don_segmentation.cpp
 * Difference of Normals Example for PCL Segmentation Tutorials.
 *
 * @author Yani Ioannou
 * @date 2012-09-24
 */
#include <string>

#include <pcl17/point_types.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/search/organized.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/features/normal_3d_omp.h>
#include <pcl17/filters/conditional_removal.h>
#include <pcl17/segmentation/extract_clusters.h>

#include <pcl17/features/don.h>

using namespace pcl17;
using namespace std;

int
main (int argc, char *argv[])
{
  ///The smallest scale to use in the DoN filter.
  double scale1;

  ///The largest scale to use in the DoN filter.
  double scale2;

  ///The minimum DoN magnitude to threshold by
  double threshold;

  ///segment scene into clusters with given distance tolerance using euclidean clustering
  double segradius;

  if (argc < 6)
  {
    cerr << "usage: " << argv[0] << " inputfile smallscale largescale threshold segradius" << endl;
    exit (EXIT_FAILURE);
  }

  /// the file to read from.
  string infile = argv[1];
  /// small scale
  istringstream (argv[2]) >> scale1;
  /// large scale
  istringstream (argv[3]) >> scale2;
  istringstream (argv[4]) >> threshold;   // threshold for DoN magnitude
  istringstream (argv[5]) >> segradius;   // threshold for radius segmentation

  // Load cloud in blob format
  sensor_msgs::PointCloud2 blob;
  pcl17::io::loadPCDFile (infile.c_str (), blob);
  pcl17::PointCloud<PointXYZRGB>::Ptr cloud (new pcl17::PointCloud<PointXYZRGB>);
  pcl17::fromROSMsg (blob, *cloud);

  // Create a search tree, use KDTreee for non-organized data.
  pcl17::search::Search<PointXYZRGB>::Ptr tree;
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl17::search::OrganizedNeighbor<PointXYZRGB> ());
  }
  else
  {
    tree.reset (new pcl17::search::KdTree<PointXYZRGB> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);

  if (scale1 >= scale2)
  {
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit (EXIT_FAILURE);
  }

  // Compute normals using both small and large scales at each point
  pcl17::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  cout << "Calculating normals for scale..." << scale1 << endl;
  pcl17::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl17::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  cout << "Calculating normals for scale..." << scale2 << endl;
  pcl17::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl17::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl17::PointCloud<PointNormal>);
  copyPointCloud<PointXYZRGB, PointNormal>(*cloud, *doncloud);

  cout << "Calculating DoN... " << endl;
  // Create DoN operator
  pcl17::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
  pcl17::PCDWriter writer;
  writer.write<pcl17::PointNormal> ("don.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

  // Build the condition for filtering
  pcl17::ConditionOr<PointNormal>::Ptr range_cond (
    new pcl17::ConditionOr<PointNormal> ()
    );
  range_cond->addComparison (pcl17::FieldComparison<PointNormal>::ConstPtr (
                               new pcl17::FieldComparison<PointNormal> ("curvature", pcl17::ComparisonOps::GT, threshold))
                             );
  // Build the filter
  pcl17::ConditionalRemoval<PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  pcl17::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl17::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

  writer.write<pcl17::PointNormal> ("don_filtered.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

  pcl17::search::KdTree<PointNormal>::Ptr segtree (new pcl17::search::KdTree<PointNormal>);
  segtree->setInputCloud (doncloud);

  std::vector<pcl17::PointIndices> cluster_indices;
  pcl17::EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance (segradius);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (segtree);
  ec.setInputCloud (doncloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl17::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  {
    pcl17::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl17::PointCloud<PointNormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    {
      cloud_cluster_don->points.push_back (doncloud->points[*pit]);
    }

    cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;

    //Save cluster
    cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
    stringstream ss;
    ss << "don_cluster_" << j << ".pcd";
    writer.write<pcl17::PointNormal> (ss.str (), *cloud_cluster_don, false);
  }
}
