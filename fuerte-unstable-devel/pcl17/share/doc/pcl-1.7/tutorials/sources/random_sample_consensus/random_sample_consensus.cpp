#include <iostream>
#include <pcl17/console/parse.h>
#include <pcl17/filters/extract_indices.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>
#include <pcl17/sample_consensus/ransac.h>
#include <pcl17/sample_consensus/sac_model_plane.h>
#include <pcl17/sample_consensus/sac_model_sphere.h>
#include <pcl17/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

boost::shared_ptr<pcl17::visualization::PCLVisualizer>
simpleVis (pcl17::PointCloud<pcl17::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer (new pcl17::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl17::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int
main(int argc, char** argv)
{
  // initialize PointClouds
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>);
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr final (new pcl17::PointCloud<pcl17::PointXYZ>);

  // populate our PointCloud with points
  cloud->width    = 500;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    if (pcl17::console::find_argument (argc, argv, "-s") >= 0 || pcl17::console::find_argument (argc, argv, "-sf") >= 0)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if (i % 5 == 0)
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else if(i % 2 == 0)
        cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                      - (cloud->points[i].y * cloud->points[i].y));
      else
        cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                        - (cloud->points[i].y * cloud->points[i].y));
    }
    else
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if( i % 2 == 0)
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else
        cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
    }
  }

  std::vector<int> inliers;

  // created RandomSampleConsensus object and compute the appropriated model
  pcl17::SampleConsensusModelSphere<pcl17::PointXYZ>::Ptr
    model_s(new pcl17::SampleConsensusModelSphere<pcl17::PointXYZ> (cloud));
  pcl17::SampleConsensusModelPlane<pcl17::PointXYZ>::Ptr
    model_p (new pcl17::SampleConsensusModelPlane<pcl17::PointXYZ> (cloud));
  if(pcl17::console::find_argument (argc, argv, "-f") >= 0)
  {
    pcl17::RandomSampleConsensus<pcl17::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }
  else if (pcl17::console::find_argument (argc, argv, "-sf") >= 0 )
  {
    pcl17::RandomSampleConsensus<pcl17::PointXYZ> ransac (model_s);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }

  // copies all inliers of the model computed to another PointCloud
  pcl17::copyPointCloud<pcl17::PointXYZ>(*cloud, inliers, *final);

  // creates the visualization object and adds either our orignial cloud or all of the inliers
  // depending on the command line arguments specified.
  boost::shared_ptr<pcl17::visualization::PCLVisualizer> viewer;
  if (pcl17::console::find_argument (argc, argv, "-f") >= 0 || pcl17::console::find_argument (argc, argv, "-sf") >= 0)
    viewer = simpleVis(final);
  else
    viewer = simpleVis(cloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
 }
