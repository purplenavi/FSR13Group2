#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>

/*void callback (const sensor_msgs::LaserScan msg)
{
    std::cout << "Test";
    ROS_INFO("min: [%.2f]", msg.range_min);
}*/

typedef enum {
    init,
    driving,
    turning,
    done,
} DriveState;

class RobotDriver
{
private:
    ros::NodeHandle nh_;

    ros::Publisher cmd_vel_pub_;

    ros::Subscriber sub_;
    
    DriveState drive_state;

public:
    RobotDriver(ros::NodeHandle &nh)
    {
        nh_ = nh;
        
        //Publisher to publish messages to cmd_vel_pub
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
        
        //Subscriber for callback
        sub_ = nh_.subscribe("scan", 1000, &RobotDriver::callback, this);
        
        drive_state = init;
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

    void callback (const sensor_msgs::LaserScan msg)
    {
        if (drive_state == done) {
            return;
        }
        
        
        //Counting laser front, left, right
        float amount = ( msg.angle_max - msg.angle_min ) / msg.angle_increment;
        int index = amount / 2;
        int left = amount / 6;
        int right = amount - left;
        
        std::cout << "Distance: " << msg.ranges[index] << ", " << msg.ranges[left] << ", " << msg.ranges[right] << ", state: " << drive_state << std::endl;
        
        if (drive_state == init) {
            this->drive();
        }
        
        else if (msg.ranges[index] < 0.5f) {
            this->stop();
        }
        
        else if (drive_state == driving && (msg.ranges[index] < 1.0f ||
           msg.ranges[left] < 0.3f ||
           msg.ranges[right] < 0.3f )) {
       
            this->turn();
        }
        
        else if (drive_state == turning && msg.ranges[index] > 2.0f &&
            msg.ranges[left] > 0.3f &&
            msg.ranges[right] > 0.3f) {
            
            this->drive();
        }
   }
   
   void drive ()
   {
       //Drive function. No turning
       geometry_msgs::Twist base_cmd;
       
       base_cmd.linear.x = 0.25;
       base_cmd.angular.z = 0.00;
       //publish the drive command
       cmd_vel_pub_.publish(base_cmd);
       
       drive_state = driving;
   }
   
   void turn ()
   {
       geometry_msgs::Twist base_cmd;
       
       base_cmd.linear.x = 0.0;
       base_cmd.angular.z = 0.75;
       //Publish turn command
       cmd_vel_pub_.publish(base_cmd);
       
       drive_state = turning;
   }
   
   void stop ()
   {
       geometry_msgs::Twist base_cmd;
       
       base_cmd.linear.x = 0.0;
       base_cmd.angular.z = 0.0;
       //Stop the robot
       cmd_vel_pub_.publish(base_cmd);
       
       drive_state = done;
       std::cout << "Done!\n";
       
       ros::shutdown();
   }
   

};

int main(int argc, char** argv)
{
    //init ROS node
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;

    RobotDriver driver(nh);
    
    //driver.driveKeyboard();
    //Loop
    ros::spin();
    
}
