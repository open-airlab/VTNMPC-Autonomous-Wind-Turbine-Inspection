#ifndef _MOCAP_GAZEBO_H
#define _MOCAP_GAZEBO_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>

// messages
gazebo_msgs::ModelStates gazebo_modelpos;
gazebo_msgs::LinkStates gazebo_linkpos;
geometry_msgs::PoseStamped mocap_pos_out;
geometry_msgs::TwistStamped mocap_vel, mocap_vel_body;

// Subscribers
ros::Subscriber gazebo_modelpos_sub;
ros::Subscriber gazebo_linkpos_sub;

// Publishers
ros::Publisher pos_mocap_pub;
ros::Publisher vel_mocap_pub;
ros::Publisher vel_body_mocap_pub;
ros::Publisher omegas_pub;

int filter_size;
std::vector<double> mocap_vel_x_unfiltered, mocap_vel_y_unfiltered, mocap_vel_z_unfiltered;

double roll_out, pitch_out, yaw_out;

int model_idx;
std::vector<int> omegas_idx;
std::vector<double> omegas_vec;

#endif
