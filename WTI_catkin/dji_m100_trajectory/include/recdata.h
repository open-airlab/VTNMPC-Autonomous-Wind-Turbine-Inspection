#include <fstream>
#include <math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <dji_m100_trajectory/set_recdataConfig.h>

using namespace std;
using namespace ros;
using namespace Eigen;

const double rad2deg = 180/M_PI;

// Other Subscribers
ros::Subscriber trajectory_start_sub, ref_trajectory_sub, ref_trajectory_delay_sub, ref_velocity_sub, point_sub, normal_sub;

// UAV feedback subscribers
ros::Subscriber pos_att_sub, local_vel_rates_sub;

// Wind subscriber
ros::Subscriber wind_commanded_sub, dist_Fx_mu_sub, dist_Fy_mu_sub, dist_Fz_mu_sub, dist_Fx_var_sub, dist_Fy_var_sub, dist_Fz_var_sub;

// NMPC subscribers
ros::Subscriber nmpc_rpy_sub, nmpc_Fz_sub, nmpc_exeTime_sub, nmpc_kkt_sub, nmpc_obj_sub;
// NMHE subscribers
ros::Subscriber nmhe_vel_sub, nmhe_exeTime_sub, nmhe_kkt_sub;

// Callback variables
std_msgs::Bool trajectory_start_flag;
Vector3d ref_trajectory;
Vector3d ref_trajectory_delay;
Vector3d ref_velocity;
Vector3d current_pos,current_att;
Vector3d point,normal;
Vector3d current_vel,current_rates;
Vector3d nmpc_ryp;
Vector2d nmpc_Fz;
double nmpc_exeTime, nmpc_kkt, nmpc_obj;
Vector3d nmhe_uvw;
double nmhe_exeTime, nmhe_kkt;
Vector3d wind_commanded;
std::vector<double> dist_Fx_mu, dist_Fy_mu, dist_Fz_mu;
std::vector<double> dist_Fx_var, dist_Fy_var, dist_Fz_var;

ifstream read_name, read_results;
ofstream print_results;

bool start_rec_cmd, stop_rec_cmd, rec_Fx_dist, rec_Fy_dist, rec_Fz_dist;
int rec_config;
double pred_horizon, max_rec_time;

std::vector<double> current_pos_att;
std::vector<double> current_vel_rate;

double t, t_last, traj_time;
bool rec_started_flag = 0;

int print_rec_start_flag = 1, print_start_rec_cmd_flag = 1;
