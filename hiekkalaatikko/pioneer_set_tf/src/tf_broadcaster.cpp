#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

static const char NODE[] = "tf_broadcaster";
//Theres no rotation
static const tf::Quaternion ROTATION(0, 0, 0, 1);
//10cm forward 20cm up
static const tf::Vector3 TRANSLATION(0.1, 0.0, 0.2);


int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE);
    ros::NodeHandle n;

    //100 Hz rate
    ros::Rate r(100);
    tf::TransformBroadcaster broadcaster;

    while (n.ok())
    {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(ROTATION, TRANSLATION),
                ros::Time::now(),
                "base_link",
                "sonar_frame"));

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(ROTATION, TRANSLATION),
                ros::Time::now(),
                "base_link",
                "laser"));

        r.sleep();
    }
}
