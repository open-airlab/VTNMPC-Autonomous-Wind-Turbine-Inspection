/**
 * @file   gp_disturb_main.cpp
 * @author Mohit Mehndiratta
 * @date   April 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include "gp_disturb_main.h"

using namespace libgp;
using namespace std;

double sampleTime = 0.05;

// Callback functions
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_msg = *msg;
}
void regression_on_cb(const std_msgs::Bool::ConstPtr& msg)
{
    regression_on_msg = *msg;
}
void local_acc_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    current_acc = {msg->linear_acceleration.x,
                   msg->linear_acceleration.y,
                   msg->linear_acceleration.z};

    // filtering
    current_acc = acc_filter(current_acc);
}
void local_vel_rates_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_rates = {msg->twist.linear.x, msg->twist.linear.y,
                         msg->twist.linear.z, msg->twist.angular.x,
                         msg->twist.angular.y, msg->twist.angular.z};
}
void nmpc_cmd_rpy_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    nmpc_cmd_rpy.clear();
    int i = 0;
    for(std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmpc_cmd_rpy.push_back(*itr);
        i++;
    }
}
void nmpc_cmd_Fz_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    nmpc_cmd_Fz.clear();
    int i = 0;
    for(std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmpc_cmd_Fz.push_back(*itr);
        i++;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gp_disturb_reg");
    ros::NodeHandle nh;

    // ----------
    // Subscribers
    // ----------
    ros::param::get("mocap_topic_part", mocap_topic_part);
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
    regression_on_sub = nh.subscribe<std_msgs::Bool>("/regression_on", 1, regression_on_cb);
    local_acc_sub  = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, local_acc_cb);
    local_vel_rates_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/" + mocap_topic_part + "/velocity_body", 1, local_vel_rates_cb);
//    local_vel_rates_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/mocap_gps/velocity", 1, local_vel_rates_cb);
    nmpc_cmd_rpy_sub = nh.subscribe<std_msgs::Float64MultiArray>("/outer_nmpc_cmd/rpy", 1, nmpc_cmd_rpy_cb);
    nmpc_cmd_Fz_sub = nh.subscribe<std_msgs::Float64MultiArray>("/outer_nmpc_cmd/Fz_FzScaled", 1, nmpc_cmd_Fz_cb);

    ros::Rate rate(1/sampleTime);

    int print_flag_offboard = 1;
    int print_flag_arm = 1;
    int print_flag_altctl = 1;

    // Roslaunch parameters
    ros::param::get("switch_xyz",switch_xyz);
    GP_DISTURB_REG::data_struct_ datastruct;
//    datastruct.m = 1.3;           // Quad_F330
//    datastruct.m = 1.412;           // Talon tricpoter
    datastruct.m = 4.2;           // Quad_matrice_100
    datastruct.g = 9.81;
    datastruct.input_0 = 0;
    datastruct.mu_0 = 0;
    datastruct.var_0 = 100;
    ros::param::get("predInit_pub_topic",datastruct.predInit_pub_topic);
    ros::param::get("mu_pub_topic",datastruct.mu_pub_topic);
    ros::param::get("var_pub_topic",datastruct.var_pub_topic);
    ros::param::get("mu_p_2std_dev_topic",datastruct.mu_p_2std_dev_topic);
    ros::param::get("mu_m_2std_dev_topic",datastruct.mu_m_2std_dev_topic);

    GP_DISTURB_REG::gp_struct_ gpstruct;
    ros::param::get("file_gp_initial",gpstruct.file_gp_initial);
    gpstruct.file_gp_initial = ros::package::getPath("gp_regression") + "/data/gp/" + gpstruct.file_gp_initial;
    ros::param::get("create_new_gp_initial",gpstruct.create_new_gp_initial);
    ros::param::get("use_gp_initial",gpstruct.use_gp_initial);
    ros::param::get("gp_type",gpstruct.gp_type);
    int data_history, num_window_points_past, num_predict_points_future;
    ros::param::get("data_history",data_history);
    gpstruct.data_history = data_history;
    ros::param::get("max_initial_rec_time_sec",gpstruct.max_initial_rec_time_sec);
    ros::param::get("num_window_points_past",num_window_points_past);
    gpstruct.num_window_points_past = num_window_points_past;
    ros::param::get("num_predict_points_future",num_predict_points_future);
    gpstruct.num_predict_points_future = num_predict_points_future;
    ros::param::get("rec_rate_factor",gpstruct.rec_rate_factor);
    ros::param::get("use_sparse_gp_initial",gpstruct.use_sparse_gp_initial);
    ros::param::get("sizefactor_sparse_gp_initial",gpstruct.sizefactor_sparse_gp_initial);
    if (gpstruct.sizefactor_sparse_gp_initial > 1)
    {
        ROS_WARN("Value greater than 1 is input for sizefactor_sparse_gp_initial.");
        ROS_WARN("sizefactor_sparse_gp_initial = 1 is enforced.");
        gpstruct.sizefactor_sparse_gp_initial = 1;
    }
    switch (gpstruct.gp_type) {
    case 0:
        gpstruct.ell_0.setConstant(gpstruct.data_history, 1.0);
        gpstruct.sx_0.setConstant(gpstruct.data_history, 0.0 * 0.0);
        break;

    case 1:
        gpstruct.ell_0.setConstant(2*gpstruct.data_history, 1.0);
        gpstruct.sx_0.setConstant(2*gpstruct.data_history, 0.0 * 0.0);
        break;
    }
    gpstruct.sf_0 = 1;
    gpstruct.sn_0 = 1;
    ros::param::get("grad_updates",gpstruct.grad_updates);
    ros::param::get("covfun1",gpstruct.covfun1);
    ros::param::get("covfun2",gpstruct.covfun2);
    ros::param::get("hyper_optimize_method",gpstruct.hyper_optimize_method);

//    gpstruct.file_gp_initial = ros::package::getPath("gp_regression") + "/data/gp/gp_initial_x.dat";
//    gpstruct.create_new_gp_initial = true;
//    gpstruct.data_history = 5;
//    gpstruct.max_initial_rec_time_sec = 15;
//    gpstruct.num_window_points_past = 200;
//    gpstruct.num_predict_points_future = 31;
//    gpstruct.rec_rate = 0.5/sampleTime;
//    gpstruct.use_sparse_gp_initial = false;
//    gpstruct.ell_0.setConstant(gpstruct.data_history, 1.0);
//    gpstruct.sf_0 = 1;
//    gpstruct.sn_0 = 1;
//    gpstruct.sx_0.setConstant(gpstruct.data_history, 0.0 * 0.0);
//    gpstruct.grad_updates = 200;
////    gpstruct.covfun1 = "CovSEiso";
//    gpstruct.covfun1 = "CovSEard";
//    gpstruct.covfun2 = "CovNoise";
//    gpstruct.hyper_optimize_method = "CG";
////    gpstruct.hyper_optimize_method = "RProp";

    // variables for acceleration filter
    current_acc.resize(3, 0.0);
    current_acc_unfiltered.resize(1, current_acc);          // size 1 means no filtering
    current_acc_filtered.resize(3, 0.0);

    bool gp_obj_created = false;
    bool regression_stop = false;

    GP_DISTURB_REG *gp_disturb_reg;
    ros::Rate ros_rec_rate(gpstruct.rec_rate_factor/sampleTime);

    for (int i=0; i<(int)2/sampleTime; ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok() && !regression_stop)
    {
        t = ros::Time::now().toSec();

        if( current_state_msg.mode != "OFFBOARD" && print_flag_offboard == 1)
        {
            ROS_INFO("OFFBOARD mode is not enabled!");
            print_flag_offboard = 0;
        }
        if( !current_state_msg.armed && print_flag_arm == 1)
        {
            ROS_INFO("Vehicle is not armed!");
            print_flag_arm = 2;
        }
        else if(current_state_msg.armed && print_flag_arm == 2)
        {
            ROS_INFO("Vehicle is armed!");
            print_flag_arm = 0;
        }

        if( current_state_msg.mode == "ALTCTL")
        {
            if(print_flag_altctl == 1)
            {
                ROS_INFO("ALTCTL mode is enabled!");
                print_flag_altctl = 0;
            }
        }

        while(ros::ok() && regression_on_msg.data && !regression_stop)
        {
            if(regression_on_msg.data && print_flag_regression_on)
            {
                ROS_INFO("Regression switch turned on!");
                print_flag_regression_on = false;
            }
            if (!gp_obj_created)
            {
                gp_disturb_reg = new GP_DISTURB_REG(sampleTime, switch_xyz, datastruct, gpstruct,
                                                    ros_rec_rate, current_vel_rates, current_acc,
                                                    nmpc_cmd_rpy, nmpc_cmd_Fz);   // CHECK!!!!!!
                gp_obj_created = true;
            }

            if (!gp_disturb_reg->return_prediction_init_value())
                gp_disturb_reg->reg_init(ros_rec_rate, current_vel_rates, current_acc,
                                         nmpc_cmd_rpy, nmpc_cmd_Fz);
            // TO BE: ros_rec_rate -> rate in the above function.

            gp_disturb_reg->reg_core(current_vel_rates, current_acc, nmpc_cmd_rpy,
                                     nmpc_cmd_Fz);
            if(std::isnan(gp_disturb_reg->predict_struct.mu[0]) == true ||
               std::isnan(gp_disturb_reg->predict_struct.mu[1]) == true ||
               std::isnan(gp_disturb_reg->predict_struct.mu[2]) == true ||
               std::isnan(gp_disturb_reg->predict_struct.mu[3]) == true)
            {
                ROS_ERROR_STREAM("GP Regression ERROR at time = " << ros::Time::now().toSec() - t <<" (sec)" );
//                regression_stop = true;
                //        exit(0);
            }

            t_gp_loop = ros::Time::now().toSec() - t;
//            if (std::fmod((double)(t_gp_loop - std::floor(t_gp_loop)), (double)(1)) < sampleTime)
//            {
//                std::cout<<"loop time for gp_disturb_reg node: " << (int)t_gp_loop << " (sec)"<<"\n";
//                std::cout<<"---------------------------------------\n";
//            }

            print_flag_offboard = 1;
            print_flag_arm = 1;
            print_flag_altctl = 1;

            ros::spinOnce();
            rate.sleep();
        }

        if (gp_obj_created)
            gp_disturb_reg->publish_muVar();

        // TO BE: create a function to publish values if regression_stop == true

        if(!regression_on_msg.data && !print_flag_regression_on)
        {
            ROS_INFO("Waiting for regression switch to begin!");
            print_flag_regression_on = true;
        }
        else if(regression_on_msg.data && print_flag_regression_on)
        {
            ROS_INFO("Regression switch turned on!");
            print_flag_regression_on = false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

std::vector<double> acc_filter(std::vector<double> &_current_acc)
{
    // erase from the front!
    current_acc_unfiltered.erase(current_acc_unfiltered.begin());
    current_acc_unfiltered.push_back(_current_acc);

    std::vector<double> current_acc_filtered(_current_acc.size(), 0.0);
    for (int i=0; i<_current_acc.size(); i++)
    {
        for (int j=0; j<current_acc_unfiltered.size(); j++)
            current_acc_filtered[i] += current_acc_unfiltered[j][i];
        current_acc_filtered[i] /= current_acc_unfiltered.size();
    }

    return current_acc_filtered;
}

