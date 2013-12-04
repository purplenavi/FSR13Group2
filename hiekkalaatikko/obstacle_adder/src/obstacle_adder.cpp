#include <ros/ros.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>
#include <nav_msgs/Path.h>

ros::Publisher object_in_map_pub_;

void Callback(const nav_msgs::Path::ConstPtr& path)
{
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "pole";
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "base_link";
  cylinder_object.header.stamp = ros::Time::now();
  
  for(int i = 0; i < path->poses.size(); i++)
  {
    arm_navigation_msgs::Shape object;
    object.type = arm_navigation_msgs::Shape::CYLINDER;
    object.dimensions.resize(2);
    //Width
    object.dimensions[0] = .15;
    //Height
    object.dimensions[1] = .75;
    geometry_msgs::Pose pose;
    pose.position.x = path->poses[i].pose.position.x;
    pose.position.y = path->poses[i].pose.position.y;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    cylinder_object.shapes.push_back(object);
    cylinder_object.poses.push_back(pose);
  }
  object_in_map_pub_.publish(cylinder_object);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_adder");
    ros::NodeHandle node;
    
    object_in_map_pub_  = node.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
    
    ros::Duration(2.0).sleep();// This delay is so critical, otherwise the first published object may not be added in the collision_space by the environment_server

    ros::Subscriber sub = node.subscribe("/Dead", 1, Callback);
    
    ros::Rate rate(10.0);
    
    while (node.ok())
    {
        ros::spin();
        rate.sleep();
    }
    
    return 0;
}
