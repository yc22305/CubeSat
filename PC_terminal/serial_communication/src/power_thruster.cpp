#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <iostream>
#include <string>

int main(int argc, char **argv) 
{
        std::string ifPower_thrust;

        ros::init(argc, argv, "thrustPowered_client"); // Initializes Node Name
        ros::NodeHandle nh;
        ros::ServiceClient thrustPowered_cli = nh.serviceClient<std_srvs::SetBool> ("thrust_powered");

        std_srvs::SetBool srv;

        std::cout << "Power the thrusters? Y for yes, N for no. (Y/N): ";
        std::getline(std::cin, ifPower_thrust);
	if (ifPower_thrust == "Y")
	   srv.request.data = true;
	else {
	   if (ifPower_thrust == "N")
	      srv.request.data = false;
	   else {
              std::cout << "Please enter 'Y' or 'N' to determine whether to power the thrusters." << std::endl;
              return 0;
             }
          }

        thrustPowered_cli.call(srv);
	ROS_INFO("\033[01;33m[NOTICE]It's normal to get ERROR about none return since the arduino-compatible server response is still under development...");
	if (srv.request.data) {
           ROS_INFO("power the thruster...");
           ROS_INFO("%s", srv.response.message.c_str());
          }
        else {
           ROS_INFO("shut down the thruster...");
           ROS_INFO("%s", srv.response.message.c_str());
          }

        return 0;
}

