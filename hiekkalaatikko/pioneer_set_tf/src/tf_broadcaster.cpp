#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>

static const char NODE[] = "tf_camera_broadcaster";

static const double height1 = 0.07; //from last join to cam
static const double height2 = 0.4; //height to cam center
static const tf::Vector3 tfFromTiltJointToCam(0, 0, height1);
static const tf::Vector3 tfFromBaseToTilt(0.13, 0, height2 -height1);
static const tf::Transform tf1(tf::Quaternion(0,0,0,1), tfFromTiltJointToCam);
static tf::Transform tf2(tf::Quaternion(0,0,0,1), tfFromBaseToTilt);
//Total tf is from base to tilt x tilt to cam
static tf::Transform TF = tf1*tf2;

double getRadian(double degree)
{
    return (degree * 3.14)/180.0;
}

void publish(const tf::Transform &tf, const char *parent, const char *frame)
{
    ROS_INFO("publishing transform");
    static tf::TransformBroadcaster caster;
    caster.sendTransform(
        tf::StampedTransform(
            tf,
            ros::Time::now(),
            parent,
            frame));
}

void Callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ROS_INFO("TF callback");
    double pan = getRadian(msg->z-90.0);
    double tilt = getRadian(msg->y-90.0);
    tf2.setRotation(tf::createQuaternionFromRPY(0,tilt,pan));
    tf::Transform tmp = tf1*tf2;
    publish(tmp, "base_link", "camera_link");
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE);
    ros::NodeHandle n;
    ros::Rate r(100);
    ros::Subscriber sub = n.subscribe("ptu_servo_states", 1000, Callback);
    publish(TF, "base_link", "camera_link");
    while (n.ok())
    {
        ros::spinOnce();
        publish(TF, "base_link", "camera_link");
        r.sleep();
        
    }
    ROS_INFO("nodehandle not ok");

    return 0;

}
