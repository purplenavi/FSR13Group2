#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>
#include <ros/console.h>


ros::NodeHandle nh;
Servo s;

void messageCb( const std_msgs::String& command_msg){
  if(command_msg == "open")
    openServo();
  else if(command_msg == "close")
    closeServo();
  else
    sendError("Unknown message");
}

ros::Subscriber<std_msgs::String> sub("manipulator", &messageCb );

void setup() {
  s.attach(10);
  openServo();
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}

void openServo() {
  if(s.attached())
    s.write(0);
  else
    sendError("Servo not attached");
}

void closeServo() {
  if(s.attached())
    s.write(180);
  else
    sendError("Servo not attached");
}

void sendError(std_msgs::String msg) {
  ROS_DEBUG("Manipulator error: %s", msg);
}