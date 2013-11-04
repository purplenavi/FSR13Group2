#include <pcl17/point_cloud.h>
#include <pcl17/point_types.h>
#include <pcl17/io/openni_grabber.h>
#include <pcl17/visualization/cloud_viewer.h>

#include <pcl17/compression/octree_pointcloud_compression.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#ifdef WIN32
# define sleep(x) Sleep((x)*1000)
#endif

class SimpleOpenNIViewer
{
public:
  SimpleOpenNIViewer () :
    viewer (" Point Cloud Compression Example")
  {
  }

  void
  cloud_cb_ (const pcl17::PointCloud<pcl17::PointXYZRGBA>::ConstPtr &cloud)
  {
    if (!viewer.wasStopped ())
    {
      // stringstream to store compressed point cloud
      std::stringstream compressedData;
      // output pointcloud
      pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloudOut (new pcl17::PointCloud<pcl17::PointXYZRGBA> ());

      // compress point cloud
      PointCloudEncoder->encodePointCloud (cloud, compressedData);

      // decompress point cloud
      PointCloudDecoder->decodePointCloud (compressedData, cloudOut);


      // show decompressed point cloud
      viewer.showCloud (cloudOut);
    }
  }

  void
  run ()
  {

    bool showStatistics = true;

    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    pcl17::io::compression_Profiles_e compressionProfile = pcl17::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

    // instantiate point cloud compression for encoding and decoding
    PointCloudEncoder = new pcl17::io::OctreePointCloudCompression<pcl17::PointXYZRGBA> (compressionProfile, showStatistics);
    PointCloudDecoder = new pcl17::io::OctreePointCloudCompression<pcl17::PointXYZRGBA> ();

    // create a new grabber for OpenNI devices
    pcl17::Grabber* interface = new pcl17::OpenNIGrabber ();

    // make callback function from member function
    boost::function<void
    (const pcl17::PointCloud<pcl17::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    while (!viewer.wasStopped ())
    {
      sleep (1);
    }

    interface->stop ();

    // delete point cloud compression instances
    delete (PointCloudEncoder);
    delete (PointCloudDecoder);

  }

  pcl17::visualization::CloudViewer viewer;

  pcl17::io::OctreePointCloudCompression<pcl17::PointXYZRGBA>* PointCloudEncoder;
  pcl17::io::OctreePointCloudCompression<pcl17::PointXYZRGBA>* PointCloudDecoder;

};

int
main (int argc, char **argv)
{
  SimpleOpenNIViewer v;
  v.run ();

  return (0);
}
