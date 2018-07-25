#include "ros/ros.h" 
#include "serial_srvs/DesiredValue.h"
#include <iostream>

int main(int argc, char **argv) 
{
	float desiredV;

        ros::init(argc, argv, "desiredValue_client"); // Initializes Node Name
        ros::NodeHandle nh;
        ros::ServiceClient desiredValue_cli = nh.serviceClient<serial_srvs::DesiredValue> ("desired_value");

        serial_srvs::DesiredValue srv;
        
	std::cout << "enter the desired value (-180~180 degree): ";
        std::cin >> desiredV;
	if (desiredV > 180 | desiredV < -180) {
           std::cout << "Please enter a value between -180 and 180." << std::endl;
           return 0;
          } 
        srv.request.data = desiredV;

	desiredValue_cli.call(srv);
	ROS_INFO("\033[01;33m[NOTICE]It's normal to get ERROR about none return since the arduino-compatible server response is still under development...");
        ROS_INFO("send desired value: %f (degree)", srv.request.data);
        ROS_INFO("%s", srv.response.message.c_str());

        return 0;
}

