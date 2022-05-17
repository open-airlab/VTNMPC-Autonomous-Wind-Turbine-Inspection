#pragma once

#include <gp_disturb_reg.h>
#include <ros/package.h>

extern double sampleTime;

mavros_msgs::State current_state_msg;
std_msgs::Bool regression_on_msg;

std::vector<double> current_vel_rates;
std::vector<double> current_acc;
std::vector<std::vector<double>> current_acc_unfiltered;
std::vector<double> current_acc_filtered;

std::vector<double> nmpc_cmd_rpy;
std::vector<double> nmpc_cmd_Fz;

// Subscribers
ros::Subscriber state_sub;
ros::Subscriber regression_on_sub;
ros::Subscriber local_acc_sub;
ros::Subscriber local_vel_rates_sub;
ros::Subscriber nmpc_cmd_rpy_sub;
ros::Subscriber nmpc_cmd_Fz_sub;

double t, t_gp_loop;
bool print_flag_regression_on = false;

// Roslaunch paramters
int switch_xyz;
std::string mocap_topic_part;

std::vector<double> acc_filter(std::vector<double> &_current_acc);
