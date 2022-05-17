#include "ros/ros.h"
#include "airsim_ros_wrapper.h"
#include <ros/spinner.h>

//airsim_ros_pkgs/GimbalAngleEulerCmd /airsim_node/gimbal_angle_euler_cmd




int main(int argc, char** argv){
 ros::init(argc, argv, "gimbal_control");
 ros::NodeHandle nh;
 ros::Publisher gpub = nh.advertise<airsim_ros_pkgs::GimbalAngleEulerCmd>("/airsim_node/gimbal_angle_euler_cmd", 1000);
 //pub = nh.advertise<airsim_ros_pkgs::GimbalAngleEulerCmd>("/gimbal_angle_euler_cmd" ,100);
 ros::Rate loop_rate(10);
 
 while (ros::ok())
   {
    airsim_ros_pkgs::GimbalAngleEulerCmd msg;
    //msg.header.stamp = ros::Time::now();
    //msg.header.seq=1;
    //msg.vehicle_name=
    //msg.camera_name=
    ROS_INFO("hakim");
    msg.yaw=30;
    msg.roll=0;
    msg.pitch=0;
    gpub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
   }
 return 0;
}