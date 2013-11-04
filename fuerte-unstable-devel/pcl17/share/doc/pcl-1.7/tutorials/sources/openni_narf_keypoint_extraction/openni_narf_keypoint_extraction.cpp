/* \author Bastian Steder */

/* ---[ */
#include <iostream>
using namespace std;
#include <pcl17/io/openni_grabber.h>
#include <pcl17/range_image/range_image_planar.h>
#include <pcl17/common/common_headers.h>
#include <pcl17/visualization/range_image_visualizer.h>
#include <pcl17/visualization/pcl_visualizer.h>
#include <pcl17/features/range_image_border_extractor.h>
#include <pcl17/keypoints/narf_keypoint.h>
#include <pcl17/console/parse.h>

std::string device_id = "#1";

float angular_resolution = 0.5;
float support_size = 0.2f;
bool set_unseen_to_max_range = true;
int max_no_of_threads = 1;
float min_interest_value = 0.5;

boost::mutex depth_image_mutex,
             ir_image_mutex,
             image_mutex;
pcl17::PointCloud<pcl17::PointXYZ>::ConstPtr point_cloud_ptr;
boost::shared_ptr<openni_wrapper::DepthImage> depth_image_ptr;
boost::shared_ptr<openni_wrapper::IRImage> ir_image_ptr;
boost::shared_ptr<openni_wrapper::Image> image_ptr;

bool received_new_depth_data = false,
     received_new_ir_image   = false,
     received_new_image   = false;
struct EventHelper
{
  void
  depth_image_cb (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image)
  {
    if (depth_image_mutex.try_lock ())
    {
      depth_image_ptr = depth_image;
      depth_image_mutex.unlock ();
      received_new_depth_data = true;
    }
  }
};


void
printUsage (const char* progName)
{
  cout << "\n\nUsage: "<<progName<<" [options] [scene.pcd] <model.pcl> [model_2.pcl] ... [model_n.pcl]\n\n"
       << "Options:\n"
       << "-------------------------------------------\n"
       << "-d <device_id>  set the device id (default \""<<device_id<<"\")\n"
       << "-r <float>      angular resolution in degrees (default "<<angular_resolution<<")\n"
       << "-s <float>      support size for the interest points (diameter of the used sphere in meters)"
       <<                 " (default "<<support_size<<")\n"
       << "-i <float>      minimum interest value (0-1) (default "<<min_interest_value<<")"
       << "-t <int>        maximum number of threads to use (default "<< max_no_of_threads<<")\n"
       << "-h              this help\n"
       << "\n\n";
}

int main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl17::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl17::console::parse (argc, argv, "-d", device_id) >= 0)
    cout << "Using device id \""<<device_id<<"\".\n";
  if (pcl17::console::parse (argc, argv, "-r", angular_resolution) >= 0)
    cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
  angular_resolution = pcl17::deg2rad (angular_resolution);
  if (pcl17::console::parse (argc, argv, "-s", support_size) >= 0)
    cout << "Setting support size to "<<support_size<<"m.\n";
  if (pcl17::console::parse (argc, argv, "-i", min_interest_value) >= 0)
    cout << "Setting minimum interest value to "<<min_interest_value<<".\n";
  if (pcl17::console::parse (argc, argv, "-t", max_no_of_threads) >= 0)
    cout << "Setting maximum number of threads to "<<max_no_of_threads<<".\n";
  
  pcl17::visualization::RangeImageVisualizer range_image_widget ("Range Image");
  
  pcl17::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.addCoordinateSystem (1.0f);
  viewer.setBackgroundColor (1, 1, 1);
  
  viewer.initCameraParameters ();
  viewer.setCameraPosition (0.0, -0.3, -2.0,
                            0.0, -0.3, 1.0,
                            0.0, -1.0, 0.0);
  
  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx)
           << ", product: " << driver.getProductName (deviceIdx) << ", connected: "
           << (int) driver.getBus (deviceIdx) << " @ " << (int) driver.getAddress (deviceIdx)
           << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'\n";
    }
  }
  else
  {
    cout << "\nNo devices connected.\n\n";
    return 1;
  }
  
  pcl17::Grabber* interface = new pcl17::OpenNIGrabber (device_id);
  EventHelper event_helper;
  
  boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&) > f_depth_image =
    boost::bind (&EventHelper::depth_image_cb, &event_helper, _1);
  boost::signals2::connection c_depth_image = interface->registerCallback (f_depth_image);
  
  cout << "Starting grabber\n";
  interface->start ();
  cout << "Done\n";
  
  boost::shared_ptr<pcl17::RangeImagePlanar> range_image_planar_ptr (new pcl17::RangeImagePlanar);
  pcl17::RangeImagePlanar& range_image_planar = *range_image_planar_ptr;
  
  pcl17::RangeImageBorderExtractor range_image_border_extractor;
  pcl17::NarfKeypoint narf_keypoint_detector;
  narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
  narf_keypoint_detector.getParameters ().support_size = support_size;
  narf_keypoint_detector.getParameters ().max_no_of_threads = max_no_of_threads;
  narf_keypoint_detector.getParameters ().min_interest_value = min_interest_value;
  //narf_keypoint_detector.getParameters ().calculate_sparse_interest_image = false;
  //narf_keypoint_detector.getParameloadters ().add_points_on_straight_edges = true;
  
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr keypoints_cloud_ptr (new pcl17::PointCloud<pcl17::PointXYZ>);
  pcl17::PointCloud<pcl17::PointXYZ>& keypoints_cloud = *keypoints_cloud_ptr;
  
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    range_image_widget.spinOnce ();  // process GUI events
    pcl17_sleep (0.01);
    
    bool got_new_range_image = false;
    if (received_new_depth_data && depth_image_mutex.try_lock ())
    {
      received_new_depth_data = false;
      
      //unsigned long time_stamp = depth_image_ptr->getTimeStamp ();
      //int frame_id = depth_image_ptr->getFrameID ();
      const unsigned short* depth_map = depth_image_ptr->getDepthMetaData ().Data ();
      int width = depth_image_ptr->getWidth (), height = depth_image_ptr->getHeight ();
      float center_x = width/2, center_y = height/2;
      float focal_length_x = depth_image_ptr->getFocalLength (), focal_length_y = focal_length_x;
      float original_angular_resolution = asinf (0.5f*float (width)/float (focal_length_x)) / (0.5f*float (width));
      float desired_angular_resolution = angular_resolution;
      range_image_planar.setDepthImage (depth_map, width, height, center_x, center_y,
                                        focal_length_x, focal_length_y, desired_angular_resolution);
      depth_image_mutex.unlock ();
      got_new_range_image = !range_image_planar.points.empty ();
    }
    
    if (!got_new_range_image)
      continue;
    
    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    if (set_unseen_to_max_range)
      range_image_planar.setUnseenToMaxRange ();
    narf_keypoint_detector.setRangeImage (&range_image_planar);
    pcl17::PointCloud<int> keypoint_indices;
    double keypoint_extraction_start_time = pcl17::getTime();
    narf_keypoint_detector.compute (keypoint_indices);
    double keypoint_extraction_time = pcl17::getTime()-keypoint_extraction_start_time;
    std::cout << "Found "<<keypoint_indices.points.size ()<<" key points. "
              << "This took "<<1000.0*keypoint_extraction_time<<"ms.\n";
    
    // ----------------------------------------------
    // -----Show keypoints in range image widget-----
    // ----------------------------------------------
    range_image_widget.showRangeImage (range_image_planar, 0.5f, 10.0f);
    //for (size_t i=0; i<keypoint_indices.points.size (); ++i)
      //range_image_widget.markPoint (keypoint_indices.points[i]%range_image_planar.width,
                                    //keypoint_indices.points[i]/range_image_planar.width,
                                    //pcl17::visualization::Vector3ub (0,255,0));
    
    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl17::visualization::PointCloudColorHandlerCustom<pcl17::PointWithRange> color_handler_cloud
      (range_image_planar_ptr, 0, 0, 0);
    if (!viewer.updatePointCloud<pcl17::PointWithRange> (range_image_planar_ptr, color_handler_cloud, "range image"))
      viewer.addPointCloud<pcl17::PointWithRange> (range_image_planar_ptr, color_handler_cloud, "range image");
    
    keypoints_cloud.points.resize (keypoint_indices.points.size ());
    for (size_t i=0; i<keypoint_indices.points.size (); ++i)
      keypoints_cloud.points[i].getVector3fMap () =
        range_image_planar.points[keypoint_indices.points[i]].getVector3fMap ();
    pcl17::visualization::PointCloudColorHandlerCustom<pcl17::PointXYZ> color_handler_keypoints
      (keypoints_cloud_ptr, 0, 255, 0);
    if (!viewer.updatePointCloud (keypoints_cloud_ptr, color_handler_keypoints, "keypoints"))
      viewer.addPointCloud (keypoints_cloud_ptr, color_handler_keypoints, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl17::visualization::PCL17_VISUALIZER_POINT_SIZE, 7, "keypoints");
  }

  interface->stop ();
}
