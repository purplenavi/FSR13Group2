#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>


tf::TransformListener *tf_listener;
ros::Publisher pc2_publisher;

void transformCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_in)
{
  ROS_INFO("Cloud received");
  sensor_msgs::PointCloud2 pcl_out;
  std::cout << "In callback" << std::endl;
  std::string vmp;
  vmp="/base_link";
  tf_listener->waitForTransform(vmp, (*pcl_in).header.frame_id,(*pcl_in).header.stamp, ros::Duration(5.0));
  std::cout << "Waited for transform" << std::endl;
  pcl_ros::transformPointCloud(vmp, *pcl_in, pcl_out, *tf_listener);
  std::cout << "Publishing transformed pcl" << std::endl;
  pc2_publisher.publish(pcl_out);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "new_tf");
    ros::NodeHandle node;
    
    tf_listener = new tf::TransformListener();
    tf::TransformBroadcaster caster;
    tf::Transform laser;
    tf::Transform camera;
    
    pc2_publisher = node.advertise<sensor_msgs::PointCloud2>("/points_transformed", 1);
    ros::Subscriber sub = node.subscribe("/camera/depth/points", 1, transformCallback);
    laser.setOrigin(tf::Vector3(-0.1, 0.0, 0.0));
    laser.setRotation(tf::Quaternion(0.0, 0.0, 0.0));
    
    camera.setOrigin(tf::Vector3(-0.1, 0.0, 0.3));
    camera.setRotation(tf::Quaternion(0.43, 0, 0));
    
    ros::Rate rate(10.0);
    
    while (node.ok())
    {
        //caster.sendTransform(tf::StampedTransform(laser, ros::Time::now(), "base_link", "laser"));
        //caster.sendTransform(tf::StampedTransform(camera, ros::Time::now(), "base_link", "camera_link"));
        ros::spin();
        rate.sleep();
    }
    
    return 0;
}
