#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloudCb(const sensor_msgs::PointCloud2ConstPtr& inputCloud) {
    // Filter data
    sensor_msgs::PointCloud2 filteredCloud;
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    sor.setInputCloud(inputCloud);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(filteredCloud);

    // Publish data
    pub.publish(filteredCloud);
}

int main(int argc, char** argv) {
    // Initialize ROS and set the node handle
    ros::init(argc, argv, "point_cloud");
    ros::NodeHandle nh;

    // Set the ROS subscriber for input
    ros::Subscriber sub = nh.subscribe("input", 1, cloudCb);

    // Set the ROS publisher for output
    pub = nh.advertice<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();

}

