#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "waypoint.h"

static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *client=NULL;

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation_queue");
    
    //tell the action client that we want to spin a thread by default
    client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("move_base", true);
    
    //wait for the action server to come up
    while(!client->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;
    
    std::vector<Waypoint> waypoints = new std::vector<Waypoint>;
    
    // Run through waypoints
    
    // Release memory
    free(waypoints);
    
    return 0;
}
