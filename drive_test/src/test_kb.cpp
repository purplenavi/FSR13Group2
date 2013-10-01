#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class RobotDriver
{
private:
    ros::NodeHandle nh_;

    ros::Publisher cmd_vel_pub_;


public:
    RobotDriver(ros::NodeHandle &nh)
    {
        nh_ = nh;
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
    }

    bool driveKeyboard()
    {
        std::cout << "Type commmand and press enter. "
            "+ to forward, l/r to turn.\n";

        geometry_msgs::Twist base_cmd;

        char cmd[50];

        while(nh_.ok())
        {
            std::cin.getline(cmd, 50);

            if(cmd[0]!='+' && cmd[0]!='l' && cmd[0]!='r' && cmd[0]!='.')
            {
                std::cout << "unknown command: " << cmd << "\n";
                continue;
            }

            base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

            //Move forward
            if(cmd[0]=='+')
            {
                base_cmd.linear.x = 0.25;
            }

            //left Yaw + forward
            else if(cmd[0]=='l')
            {
                base_cmd.angular.z = 0.75;
                base_cmd.linear.x = 0.25;
            }

            //right yaw + forward 
            else if(cmd[0]=='r')
            {
                base_cmd.angular.z = -0.75;
                base_cmd.linear.x = 0.25;
            }

            //quit
            else if(cmd[0]=='.')
            {
                base_cmd.angular.z = 0.00;
                base_cmd.linear.x = 0.00;
                cmd_vel_pub_.publish(base_cmd);
                break;
            }

            //publish the command
            cmd_vel_pub_.publish(base_cmd);
        }
        return true;
    }
};


int main(int argc, char** argv)
{
    //init ROS node
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;

    RobotDriver driver(nh);
    driver.driveKeyboard();
}
