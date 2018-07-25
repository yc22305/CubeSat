#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <unistd.h>
#include <math.h>
#include <iomanip>

#define BAUD 9600
#define PI 3.1415926535897932384626433832795

int main(int argc, char **argv)
{
 double q[4] = {0}, yaw = 0, pitch = 0, roll = 0;

 /*TF displayer of connected hardware*/
 ros::init(argc, argv, "tf_displayer");
 ros::NodeHandle nh;
 ros::Rate r(10); // display rate
 ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

 visualization_msgs::Marker marker;
 marker.header.frame_id = "/world";
 marker.ns = "cubesats";
 marker.id = 0;
 marker.type = visualization_msgs::Marker::CUBE; // using a cube to represent a CubeSat.
 marker.action = visualization_msgs::Marker::ADD;
 marker.scale.x = 1;
 marker.scale.y = 1;
 marker.scale.z = 1;
 marker.color.r = 0.5;
 marker.color.g = 0.5;
 marker.color.b = 0.5;
 marker.color.a = 1; // a = 0 means transparent items.
 marker.lifetime = ros::Duration();
 
 tf::TransformBroadcaster broadcaster_refer_world;
 tf::Transform transf_refer_world;

 tf::TransformListener listener_world_cubesat;
 tf::StampedTransform transf_world_cubesat;

 /*frames compared to refer frames*/
 /*set world frame*/
 transf_refer_world.setOrigin( tf::Vector3(0, 0, 0) );
 transf_refer_world.setRotation( tf::Quaternion(1, 0, 0, 0) );

 while (ros::ok()) {
     broadcaster_refer_world.sendTransform(tf::StampedTransform(transf_refer_world, ros::Time::now(), "/refer", "/world"));
     
     try {
        listener_world_cubesat.lookupTransform("/world", "/cubesat", ros::Time(0), transf_world_cubesat);
       }
     catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
       }

    q[1] = transf_world_cubesat.getRotation().x();
    q[2] = transf_world_cubesat.getRotation().y();
    q[3] = transf_world_cubesat.getRotation().z();
    q[0] = transf_world_cubesat.getRotation().w();

    yaw   = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);  
    pitch = -asin(2 * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    yaw *= 180 / PI;
    pitch   *= 180 / PI; 
    roll  *= 180 / PI; 
    std::cout << std::fixed << std::setprecision(4) << "yaw: " << std::setw(10) << yaw
    << "   pitch: " << std::setw(10) << pitch 
    << "   roll: " << std::setw(10) << roll << std::endl;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now(); // to record the present time
    marker.pose.position.x = transf_world_cubesat.getOrigin().x(); 
    marker.pose.position.y = transf_world_cubesat.getOrigin().y();
    marker.pose.position.z = transf_world_cubesat.getOrigin().z();
    marker.pose.orientation.x = q[1];
    marker.pose.orientation.y = q[2];
    marker.pose.orientation.z = q[3];
    marker.pose.orientation.w = q[0];

    /*while (marker_pub.getNumSubscribers() < 1) { // if no subscribers are set in rviz, warning occurs
       if (!ros::ok())
          return 0;
       ROS_WARN_ONCE("Please create a subscriber (in rviz) to the marker");
       sleep(1);
      }*/
    marker_pub.publish(marker);

    r.sleep();   
   }
   
 return 0;
}
