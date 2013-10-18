#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

static const char NODE[] = "stopper";
static const char CMD_VEL_TOPIC[] = "/RosAria/cmd_vel";


static bool stop = true;

void Callback(const geometry_msgs::Twist::ConstPtr& msg);


int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE);

    ros::NodeHandle n;

    ros::Rate r(1);

    ros::Subscriber cmd_vel_sub = n.subscribe(CMD_VEL_TOPIC, 1, Callback);
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, 1000);

    //When initialized, all values are zero
    geometry_msgs::Twist no_velocity;

    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        if  (stop)
        {
            ROS_ERROR("Did not see %s topic published. Avoiding crash.", CMD_VEL_TOPIC);
            cmd_vel_pub.publish(no_velocity);
        }
        else
        {
            stop = true;
        }
    }
    return 0;
}

void Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_DEBUG("cmd_vel published");
    stop = false;
}
