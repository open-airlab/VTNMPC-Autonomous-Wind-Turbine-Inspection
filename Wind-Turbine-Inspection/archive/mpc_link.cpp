

#include "ros/ros.h"
#include <ros/spinner.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/spinner.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <cmath>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>


tf2::Quaternion quaternion_;
mavros_msgs::State current_state;
mavros_msgs::Thrust T;
geometry_msgs::PoseStamped att;
mavros_msgs::AttitudeTarget att_raw;
nav_msgs::Odometry current_pose_g;



float roll_d;
float pitch_d;
float thrust_r;
float yaw_rate_d;
float psi;


float quat_x_d;
float quat_y_d;
float quat_z_d;
float quat_w_d;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}



void att_cb(const mav_msgs::RollPitchYawrateThrust::ConstPtr& msg)
   {
     roll_d= msg->roll;
     pitch_d=msg->pitch;
     thrust_r=msg->thrust.z;
    
     yaw_rate_d=msg->yaw_rate;
 }



void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_g = *msg;
  float q0 = current_pose_g.pose.pose.orientation.w;
  float q1 = current_pose_g.pose.pose.orientation.x;
  float q2 = current_pose_g.pose.pose.orientation.y;
  float q3 = current_pose_g.pose.pose.orientation.z;
  psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );

}




void euler2quat(float a1, float a2, float a3)
   {
    quaternion_.setRPY(a1,a2,a3);
    quaternion_ = quaternion_.normalize();
   att.pose.orientation.x = quaternion_.x();
   att.pose.orientation.y= quaternion_.y();
   att.pose.orientation.z = quaternion_.z();
   att.pose.orientation.w = quaternion_.w(); 



 }





int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_link_node");
    ros::NodeHandle nh;

    
    T.thrust=0.8;


    ros::Subscriber attitude_sub = nh.subscribe<mav_msgs::RollPitchYawrateThrust>("/firefly/command/roll_pitch_yawrate_thrust", 1000,att_cb);
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 1000,pose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //        ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher thrust_pub = nh.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 1000);      
    

    ros::Publisher attitude_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1000);


    ros::Publisher attitude_raw_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1000);


    

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(200.0);
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        //thrust_pub.publish(T);
        //attitude_pub.publish(att);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    
    geometry_msgs::Vector3 body_rate;
    body_rate.x = 0.0;
    body_rate.y = 0.0;
    body_rate.z = 0.0;

 
   // att_raw.body_rate.x = 0.0;
   // att_raw.body_rate.y = 0.0;
   // att_raw.body_rate.z = 0.0;


    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        euler2quat(roll_d,pitch_d,psi);
        att_raw.orientation.x = quaternion_.x();
        att_raw.orientation.y= quaternion_.y();
        att_raw.orientation.z = quaternion_.z();
        att_raw.orientation.w = quaternion_.w(); 
        //att_raw.thrust=thrust_r/50;
        att_raw.thrust=thrust_r/22;
        att.header.stamp = ros::Time::now();
        T.header.stamp = ros::Time::now();
        att_raw.header.stamp = ros::Time::now();
        //thrust_pub.publish(T);
        //attitude_pub.publish(att);
        attitude_raw_pub.publish(att_raw);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
