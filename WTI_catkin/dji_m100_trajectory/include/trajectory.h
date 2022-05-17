#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <Eigen/Dense>
#include <math.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>

#include <dynamic_reconfigure/server.h>


using namespace geometry_msgs;
using namespace std;
using namespace ros;

const double deg2rad = M_PI/180;

// Roslaunch param
std::string lidar_topic;

// Publishers
ros::Publisher ref_pos_pub;
ros::Publisher ref_pos_delay_pub;
ros::Publisher ref_vel_pub;
ros::Publisher ref_yaw_pub;
ros::Publisher setpoint_pos_pub;
ros::Publisher servo_pub;
ros::Publisher traj_on_pub;

// Subscriber
ros::Subscriber pos_sub;
ros::Subscriber lidar_read_sub;

double max_z, x_hover, y_hover, z_hover, yaw_hover, wall_dist, x_sp_start, y_sp_start, z_sp_start, x_sp_end, y_sp_end, z_sp_end, z_interval_dist, z_interval_dist_updated, del_z, radius, absvel, rotvel, time_period, climb_rate, land_rate;
int pos_pub_delay, traj_type, wall_direc, num_turns, sp_z_counter, const_z;
bool traj_start, max_z_start, lidar_start, climb_flag, land_flag, change_z, pub_setpoint_pos, use_current_pos;
bool sp_z_counter_switch, sp_left_corner_reached_flag, sp_right_corner_reached_flag;



geometry_msgs::PoseStamped pos_ref_start_msg;
geometry_msgs::Vector3 reftrajectory_msg, reftrajectory_delay_msg, reftrajectory_vel_msg;
std_msgs::Float64 ref_yaw;
geometry_msgs::PoseStamped setpoint_pos_msg;
tf::Quaternion setpoint_att_quat;
tf::Matrix3x3 R_BI, R_IB;
std_msgs::Bool traj_start_msg;

double x,y,z;
double x_B,y_B;
double x_last = 0, y_last = 0, z_last = 0;
double x_atTrajStart = 0, y_atTrajStart = 0, z_atTrajStart = 0, x_B_atTrajStart = 0, y_B_atTrajStart = 0;
double x_B_sp_start,y_B_sp_start, x_B_sp_end,y_B_sp_end;
double u,v,w;
bool x_delay_started = false, y_delay_started = false, z_delay_started = false;
double x_delay_start,y_delay_start,z_delay_start;
double x_delay,y_delay,z_delay;
double v_d;
double t, t_last, traj_time, t_last_eachRun, traj_time_eachRun, traj_time_z, t_last_z;
bool traj_started_flag = 0, climbed_flag = 0, landed_flag = 0, lidar_started_flag = 0;
int print_flag_traj_start = 0, print_flag_hover_origin = 1, print_flag_hover = 1, print_flag_hover_lidar = 1,
print_flag_circle = 1, print_flag_fig8 = 1, print_flag_square = 1, print_flag_setpoint = 1,  print_flag_changez = 1;
int print_flag_climb = 0, print_flag_land = 0;
