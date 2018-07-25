// this node is used to monitor the cubesat status instead of recording to txt file.
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

void cubeMode_msgCallback(const std_msgs::Int16::ConstPtr& msg) // used to receive any string message
{
 if (msg->data == -1)
    ROS_INFO("Fails to connect to IMU. Please check the wiring.");
 if (msg->data == 0)
    ROS_INFO("Powers OFF thrusters.");
 if (msg->data == 1)
    ROS_INFO("Powers ON thrusters.");
}

void desiredValue_msgCallback(const std_msgs::Float32::ConstPtr& msg)
{
 ROS_INFO("The desired angle is %f (degree)", msg->data);
}

void thrustSwitch_msgCallback(const std_msgs::Int16::ConstPtr& msg) // used to receive any string message
{
 ROS_INFO("The thruster status is %d", msg->data);
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "debug_subscriber");
 ros::NodeHandle nh;
 ros::Subscriber debug_desiredValue_sub = nh.subscribe("debug_desiredValue", 1, desiredValue_msgCallback);
 ros::Subscriber debug_thrustSwitch_sub = nh.subscribe("debug_thrustSwitch", 1, thrustSwitch_msgCallback);
 ros::Subscriber debug_cubeMode_sub = nh.subscribe("debug_cubeMode", 1, cubeMode_msgCallback);
 ros::Rate r(2); // display rate
 
 while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
   }

 return 0;
}
