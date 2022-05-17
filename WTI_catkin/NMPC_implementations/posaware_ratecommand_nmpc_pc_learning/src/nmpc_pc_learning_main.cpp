/**
 * @file   nmpc_pc_learning_main.cpp
 * @author Mohit Mehndiratta
 * @date   April 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <nmpc_pc_learning_main.h>

using namespace Eigen;
using namespace ros;

double sampleTime = 0.02;

mavros_msgs::State current_state_msg;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_msg = *msg;
}
void ref_position_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_position << msg->x, msg->y, msg->z;
}
void ref_velocity_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_velocity << msg->x, msg->y, msg->z;
}
void ref_yaw_cb(const std_msgs::Float64::ConstPtr& msg)
{
    ref_yaw_rad = msg->data;
}
void ref_point_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ref_point = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};

    double n1 = (ref_point[0] - current_pos_att[0]);  //vector n components n=[n1;n2;n3]
    double n2 = (ref_point[1] - current_pos_att[1]);
    double norm_n = sqrt(n1 * n1 + n2 * n2) + 0.001;
    double s = (1 / norm_n) * (cos(current_pos_att[5]) * n1 + sin(current_pos_att[5]) * n2);
    // s_dot assumes px, py, pz velocities are negligible
    double s_dot =
        (1 / norm_n) *
        (-sin(current_pos_att[5]) * current_vel_rate[5] * n1 + cos(current_pos_att[5]) * (0 - current_vel_rate[0]) +
         cos(current_pos_att[5]) * current_vel_rate[5] * n2 + sin(current_pos_att[5]) * (0 - current_vel_rate[1]));

    current_s_sdot = {s, s_dot};
}
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double roll = 0, pitch = 0, yaw = 0;
    current_att_quat = {
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
    current_att_mat.setRotation(current_att_quat);
    current_att_mat.getRPY(roll, pitch, yaw);

    current_pos_att = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, roll, pitch, yaw};
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_rate = {msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z,
                        msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z};
}

void dist_Fx_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fx.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fx.predInit && dist_Fx.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fx estimates! \n";
        dist_Fx.print_predInit = 0;
    }
}
void dist_Fy_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fy.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fy.predInit && dist_Fy.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fy estimates! \n";
        dist_Fy.print_predInit = 0;
    }
}
void dist_Fz_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fz.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fz.predInit && dist_Fz.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fz estimates! \n";
        dist_Fz.print_predInit = 0;
    }
}

void dist_Fx_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (use_dist_estimates && dist_Fx.predInit)
    {
        dist_Fx.data.clear();
        dist_Fx.data.insert(dist_Fx.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fx.data = dist_Fx.data_zeros;
}
void dist_Fy_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (use_dist_estimates && dist_Fy.predInit)
    {
        dist_Fy.data.clear();
        dist_Fy.data.insert(dist_Fy.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fy.data = dist_Fy.data_zeros;
}
void dist_Fz_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (use_dist_estimates && dist_Fz.predInit)
    {
        dist_Fz.data.clear();
        dist_Fz.data.insert(dist_Fz.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fz.data = dist_Fz.data_zeros;
}

void NMPC_PC::publish_rpyFz(struct command_struct& commandstruct)
{
    mavros_msgs::Thrust att_thro_msg;
    att_thro_msg.header.frame_id = "";
    att_thro_msg.header.stamp = ros::Time::now();
    att_thro_msg.thrust = commandstruct.control_thrust_vec[1];  // scaled thrust
    att_throttle_pub.publish(att_thro_msg);

    geometry_msgs::PoseStamped attitude_msg;
    tf::Quaternion q(attitude_msg.pose.orientation.x,
                     attitude_msg.pose.orientation.y,
                     attitude_msg.pose.orientation.z,
                     attitude_msg.pose.orientation.w);
    q.setRPY(commandstruct.control_attitude_vec[0],
             commandstruct.control_attitude_vec[1],
             commandstruct.control_attitude_vec[2]);
    attitude_msg.header.frame_id = "";
    attitude_msg.header.stamp = ros::Time::now();
    attitude_msg.pose.orientation.x = q.getX();
    attitude_msg.pose.orientation.y = q.getY();
    attitude_msg.pose.orientation.z = q.getZ();
    attitude_msg.pose.orientation.w = q.getW();
    attitude_pub.publish(attitude_msg);

    std_msgs::Float64MultiArray rpy_msg;
    rpy_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rpy_msg.layout.dim[0].size = commandstruct.control_attitude_vec.size();
    rpy_msg.layout.dim[0].stride = 1;
    rpy_msg.layout.dim[0].label = "Roll, Pitch, Yaw (rad)";
    rpy_msg.data.clear();
    rpy_msg.data.insert(
        rpy_msg.data.end(), commandstruct.control_attitude_vec.begin(), commandstruct.control_attitude_vec.end());
    nmpc_cmd_rpy_pub.publish(rpy_msg);

    std_msgs::Float64MultiArray Fz_msg;
    Fz_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    Fz_msg.layout.dim[0].size = commandstruct.control_thrust_vec.size();
    Fz_msg.layout.dim[0].stride = 1;
    Fz_msg.layout.dim[0].label = "Fz (N), Fz_scaled";
    Fz_msg.data.clear();
    Fz_msg.data.insert(
        Fz_msg.data.end(), commandstruct.control_thrust_vec.begin(), commandstruct.control_thrust_vec.end());
    nmpc_cmd_Fz_pub.publish(Fz_msg);

    std_msgs::Float64 exe_time_msg;
    exe_time_msg.data = commandstruct.exe_time;
    nmpc_cmd_exeTime_pub.publish(exe_time_msg);

    std_msgs::Float64 kkt_tol_msg;
    kkt_tol_msg.data = commandstruct.kkt_tol;
    nmpc_cmd_kkt_pub.publish(kkt_tol_msg);

    std_msgs::Float64 obj_val_msg;
    obj_val_msg.data = commandstruct.obj_val;
    nmpc_cmd_obj_pub.publish(obj_val_msg);

    std_msgs::Float64MultiArray s_sdot_msg;
    s_sdot_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    s_sdot_msg.layout.dim[0].size = current_s_sdot.size();
    s_sdot_msg.layout.dim[0].stride = 1;
    s_sdot_msg.layout.dim[0].label = "s, sdot";
    s_sdot_msg.data.clear();
    s_sdot_msg.data.insert(s_sdot_msg.data.end(), current_s_sdot.begin(), current_s_sdot.end());
    s_sdot_pub.publish(s_sdot_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "poseaware_nmpc_pc_learning");
    ros::NodeHandle nh;

    ros::param::get("mocap_topic_part", mocap_topic_part);
    ros::param::get("dist_Fx_predInit_topic", dist_Fx_predInit_topic);
    ros::param::get("dist_Fy_predInit_topic", dist_Fy_predInit_topic);
    ros::param::get("dist_Fz_predInit_topic", dist_Fz_predInit_topic);
    ros::param::get("dist_Fx_data_topic", dist_Fx_data_topic);
    ros::param::get("dist_Fy_data_topic", dist_Fy_data_topic);
    ros::param::get("dist_Fz_data_topic", dist_Fz_data_topic);

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);

    ref_position_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/position", 1, ref_position_cb);
    ref_velocity_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/velocity", 1, ref_velocity_cb);
    ref_yaw_sub = nh.subscribe<std_msgs::Float64>("ref_trajectory/yaw", 1, ref_yaw_cb);
    ref_point_sub = nh.subscribe<geometry_msgs::PoseStamped>("point_to_view", 1, ref_point_cb);
    //    pos_sub = private_nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pos_cb);
    //    vel_sub = private_nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 1, vel_cb);
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/" + mocap_topic_part + "/pose", 1, pos_cb);
    //    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/" + mocap_topic_part + "/velocity", 1, vel_cb);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/" + mocap_topic_part + "/velocity_body", 1, vel_cb);
    dist_Fx_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fx_predInit_topic, 1, dist_Fx_predInit_cb);
    dist_Fy_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fy_predInit_topic, 1, dist_Fy_predInit_cb);
    dist_Fz_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fz_predInit_topic, 1, dist_Fz_predInit_cb);
    dist_Fx_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fx_data_topic, 1, dist_Fx_data_cb);
    dist_Fy_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fy_data_topic, 1, dist_Fy_data_cb);
    dist_Fz_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fz_data_topic, 1, dist_Fz_data_cb);

    // ----------
    // Publishers
    // ----------
    att_throttle_pub = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust", 1, true);
    attitude_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 1, true);
    nmpc_cmd_rpy_pub = nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/rpy", 1, true);
    nmpc_cmd_Fz_pub = nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/Fz_FzScaled", 1, true);
    nmpc_cmd_exeTime_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/exeTime", 1, true);
    nmpc_cmd_kkt_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/kkt", 1, true);
    nmpc_cmd_obj_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/obj", 1, true);

    s_sdot_pub = nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/s_sdot", 1, true);

    nmpc_struct.U_ref.resize(NMPC_NU);
    nmpc_struct.W.resize(NMPC_NY);

    // Roslaunch parameters
    ros::param::get("verbose", nmpc_struct.verbose);
    ros::param::get("yaw_control", nmpc_struct.yaw_control);
    ros::param::get("online_ref_yaw", online_ref_yaw);
    ros::param::get("use_dist_estimates", use_dist_estimates);

    ros::param::get("min_Fz_scale", nmpc_struct.min_Fz_scale);
    ros::param::get("max_Fz_scale", nmpc_struct.max_Fz_scale);
    ros::param::get("W_Wn_factor", nmpc_struct.W_Wn_factor);
    int u_idx = 0;
    ros::param::get("phi_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("theta_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("r_rate_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("Fz_ref", nmpc_struct.U_ref(u_idx++));
    nmpc_struct.U_ref(3) =
        ((nmpc_struct.max_Fz_scale - nmpc_struct.min_Fz_scale) / (1 - 0)) * (nmpc_struct.U_ref(3) - 0);
    //    std::cout<<"nmpc_struct.U_ref(3) = "<<nmpc_struct.U_ref(3)<<"\n";
    assert(u_idx == NMPC_NU);

    int w_idx = 0;
    ros::param::get("W_x", nmpc_struct.W(w_idx++));
    ros::param::get("W_y", nmpc_struct.W(w_idx++));
    ros::param::get("W_z", nmpc_struct.W(w_idx++));
    ros::param::get("W_u", nmpc_struct.W(w_idx++));
    ros::param::get("W_v", nmpc_struct.W(w_idx++));
    ros::param::get("W_w", nmpc_struct.W(w_idx++));
    ros::param::get("W_psi", nmpc_struct.W(w_idx++));
    ros::param::get("W_s", nmpc_struct.W(w_idx++));
    ros::param::get("W_sdot", nmpc_struct.W(w_idx++));
    ros::param::get("W_phi", nmpc_struct.W(w_idx++));
    ros::param::get("W_theta", nmpc_struct.W(w_idx++));
    ros::param::get("W_r_rate", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fz", nmpc_struct.W(w_idx++));
    assert(w_idx == NMPC_NY);

    nmpc_struct.sample_time = sampleTime;

    NMPC_PC* nmpc_pc = new NMPC_PC(nmpc_struct);
    ros::Rate rate(1 / sampleTime);

    current_pos_att.resize(6);
    current_vel_rate.resize(6);
    ref_point.resize(3);
    current_s_sdot.resize(2);
    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);
    dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fy.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fz.data_zeros.resize(NMPC_N + 1, 0.0);

    ref_traj_type = 0;
    ref_position << 0, 0, 0;
    ref_velocity << 0, 0, 0;

    control_stop = false;

    for (int i = 0; i < (int)(1 / sampleTime); ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && !control_stop)
    {
        t = ros::Time::now().toSec();

        if (current_state_msg.mode != "OFFBOARD" && print_flag_offboard == 1)
        {
            ROS_INFO("OFFBOARD mode is not enabled!");
            print_flag_offboard = 0;
        }
        if (!current_state_msg.armed && print_flag_arm == 1)
        {
            ROS_INFO("Vehicle is not armed!");
            print_flag_arm = 2;
        }
        else if (current_state_msg.armed && print_flag_arm == 2)
        {
            ROS_INFO("Vehicle is armed!");
            print_flag_arm = 0;
        }

        if (current_state_msg.mode == "ALTCTL")
        {
            pos_ref = current_pos_att;
            if (print_flag_altctl == 1)
            {
                ROS_INFO("ALTCTL mode is enabled!");
                print_flag_altctl = 0;
            }
        }

        if (!nmpc_pc->return_control_init_value())
        {
            nmpc_pc->nmpc_init(pos_ref, nmpc_pc->nmpc_struct);
            if (nmpc_struct.verbose && nmpc_pc->return_control_init_value())
            {
                std::cout << "***********************************\n";
                std::cout << "NMPC: initialized correctly\n";
                std::cout << "***********************************\n";
            }
        }

        while (ros::ok() && current_state_msg.mode == "OFFBOARD" && !control_stop)
        {
            if (online_ref_yaw)
            {
                nmpc_struct.ref_yaw = ref_yaw_rad;
            }
            t_cc_loop = ros::Time::now().toSec() - t;
            if (std::fmod(std::abs(t_cc_loop - (int)(t_cc_loop)), (double)(sampleTime)) == 0)
                std::cout << "loop time for outer NMPC: " << t_cc_loop << " (sec)"
                          << "\n";

            // Setting up state-feedback [x,y,z,u,v,w,psi,0,0,0,p,q]
            current_states = {current_pos_att.at(0),
                              current_pos_att.at(1),
                              current_pos_att.at(2),
                              current_vel_rate.at(0),
                              current_vel_rate.at(1),
                              current_vel_rate.at(2),
                              current_pos_att.at(5),
                              ref_point[0],
                              ref_point[1],
                              ref_point[2],
                              current_vel_rate.at(3),
                              current_vel_rate.at(4)};

            // Setting up references [x,y,z,u,v,w,psi,s,s_dot]
            ref_trajectory = {ref_position(0),
                              ref_position(1),
                              ref_position(2),
                              ref_velocity(0),
                              ref_velocity(1),
                              ref_velocity(2),
                              ref_yaw_rad,
                              0.0,
                              0.0};

            std::cout << "current_states = ";
            for (int idx = 0; idx < current_states.size(); idx++)
            {
                std::cout << current_states[idx] << ",";
            }
            std::cout << "\n";

            std::cout << "ref_trajectory = ";
            for (int idx = 0; idx < ref_trajectory.size(); idx++)
            {
                std::cout << ref_trajectory[idx] << ",";
            }
            std::cout << "\n";

            std::cout << "ref_point = ";
            for (int idx = 0; idx < ref_point.size(); idx++)
            {
                std::cout << ref_point[idx] << ",";
            }
            std::cout << "\n";

            online_data.distFx = dist_Fx.data;
            online_data.distFy = dist_Fy.data;
            online_data.distFz = dist_Fz.data;
            online_data.ref_point = ref_point;

            nmpc_pc->nmpc_core(nmpc_struct,
                               nmpc_pc->nmpc_struct,
                               nmpc_pc->nmpc_cmd_struct,
                               ref_trajectory,
                               online_data,
                               current_states);

            if (nmpc_pc->acado_feedbackStep_fb != 0)
                control_stop = true;

            if (std::isnan(nmpc_pc->nmpc_struct.u[0]) == true || std::isnan(nmpc_pc->nmpc_struct.u[1]) == true ||
                std::isnan(nmpc_pc->nmpc_struct.u[2]) == true || std::isnan(nmpc_pc->nmpc_struct.u[3]) == true)
            {
                ROS_ERROR_STREAM("Controller ERROR at time = " << ros::Time::now().toSec() - t << " (sec)");
                control_stop = true;
                exit(0);
            }

            nmpc_pc->publish_rpyFz(nmpc_pc->nmpc_cmd_struct);

            print_flag_offboard = 1;
            print_flag_arm = 1;
            print_flag_altctl = 1;

            ros::spinOnce();
            rate.sleep();
        }

        nmpc_pc->publish_rpyFz(nmpc_pc->nmpc_cmd_struct);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
