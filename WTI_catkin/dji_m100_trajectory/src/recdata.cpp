/**
 * @file   recdata.cpp
 * @author Mohit Mehndiratta
 * @date   July 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <recdata.h>

double sampleTime = 0.02;
void dynamicReconfigureCallback(dji_m100_trajectory::set_recdataConfig &config, uint32_t level)
{
    start_rec_cmd = config.start_rec;
    stop_rec_cmd = config.stop_rec;
    pred_horizon = config.pred_horizon;
    rec_config = config.rec_config;
    rec_Fx_dist = config.rec_Fx_dist;
    rec_Fy_dist = config.rec_Fy_dist;
    rec_Fz_dist = config.rec_Fz_dist;
    max_rec_time = config.max_rec_time;
}

// Other callbacks
void trajectory_start_cb(const std_msgs::Bool::ConstPtr& msg)
{
    trajectory_start_flag = *msg;
}
void ref_trajectory_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_trajectory << msg->x, msg->y, msg->z;
}










void ref_trajectory_delay_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_trajectory_delay << msg->x, msg->y, msg->z;
}
void ref_velocity_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_velocity << msg->x, msg->y, msg->z;
}

// UAV feedback callbacks
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(current_att(0),current_att(1),current_att(2));
    current_att(0) = rad2deg*current_att(0);
    current_att(1) = rad2deg*current_att(1);
    current_att(2) = rad2deg*current_att(2);

}


void point_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    point << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
}

void norm_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    normal << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
}














void local_vel_rates_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel << msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z;
    current_rates << rad2deg*msg->twist.angular.x, rad2deg*msg->twist.angular.y, rad2deg*msg->twist.angular.z;
}

// NMPC callbacks
void nmpc_rpy_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int i = 0;
    for(std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmpc_ryp(i) = *itr;
        i++;
    }
}
void nmpc_Fz_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int i = 0;
    for(std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmpc_Fz(i) = *itr;
        i++;
    }
}
void nmpc_exeTime_cb(const std_msgs::Float64::ConstPtr& msg)
{
    nmpc_exeTime = msg->data;
}
void nmpc_kkt_cb(const std_msgs::Float64::ConstPtr& msg)
{
    nmpc_kkt = msg->data;
}

void nmpc_obj_cb(const std_msgs::Float64::ConstPtr& msg)
{
    nmpc_obj = msg->data;
}


// NMHE subscribers
void nmhe_uvw_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int i = 0;
    for(std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmhe_uvw(i) = *itr;
        i++;
    }
}
void nmhe_exeTime_cb(const std_msgs::Float64::ConstPtr& msg)
{
    nmhe_exeTime = msg->data;
}
void nmhe_kkt_cb(const std_msgs::Float64::ConstPtr& msg)
{
    nmhe_kkt = msg->data;
}

// Wind subscriber
void wind_commanded_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    wind_commanded << msg->x, msg->y, msg->z;
}
void dist_Fx_mu_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    dist_Fx_mu.clear();
    dist_Fx_mu.insert(dist_Fx_mu.end(), msg->data.begin(), msg->data.end());
}
void dist_Fy_mu_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    dist_Fy_mu.clear();
    dist_Fy_mu.insert(dist_Fy_mu.end(), msg->data.begin(), msg->data.end());
}
void dist_Fz_mu_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    dist_Fz_mu.clear();
    dist_Fz_mu.insert(dist_Fz_mu.end(), msg->data.begin(), msg->data.end());
}
void dist_Fx_var_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    dist_Fx_var.clear();
    dist_Fx_var.insert(dist_Fx_var.end(), msg->data.begin(), msg->data.end());
}
void dist_Fy_var_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    dist_Fy_var.clear();
    dist_Fy_var.insert(dist_Fy_var.end(), msg->data.begin(), msg->data.end());
}
void dist_Fz_var_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    dist_Fz_var.clear();
    dist_Fz_var.insert(dist_Fz_var.end(), msg->data.begin(), msg->data.end());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "record_data");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<dji_m100_trajectory::set_recdataConfig> server;
    dynamic_reconfigure::Server<dji_m100_trajectory::set_recdataConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    // Other subscribers
    trajectory_start_sub = nh.subscribe<std_msgs::Bool>("trajectory_on", 1, trajectory_start_cb);
    ref_trajectory_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/position", 1, ref_trajectory_cb);
    ref_trajectory_delay_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/position_delayed", 1, ref_trajectory_delay_cb);
    ref_velocity_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/velocity", 1, ref_velocity_cb);

    point_sub = nh.subscribe<geometry_msgs::PoseStamped>("/point_to_view_traj", 1, point_cb);
    normal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/norm_traj", 1, norm_cb);


    // UAV feedback subscribers
    pos_att_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/mocap/pose", 1, pos_cb);
    local_vel_rates_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/mocap/velocity", 1, local_vel_rates_cb);

    // NMPC subscribers
    nmpc_rpy_sub = nh.subscribe<std_msgs::Float64MultiArray>("outer_nmpc_cmd/rpy", 1, nmpc_rpy_cb);
    nmpc_Fz_sub = nh.subscribe<std_msgs::Float64MultiArray>("outer_nmpc_cmd/Fz_FzScaled", 1, nmpc_Fz_cb);
    nmpc_exeTime_sub = nh.subscribe<std_msgs::Float64>("outer_nmpc_cmd/exeTime", 1, nmpc_exeTime_cb);
    nmpc_kkt_sub = nh.subscribe<std_msgs::Float64>("outer_nmpc_cmd/kkt", 1, nmpc_kkt_cb);
    nmpc_obj_sub = nh.subscribe<std_msgs::Float64>("outer_nmpc_cmd/obj", 1, nmpc_obj_cb);
    // Wind subscribers
    wind_commanded_sub = nh.subscribe<geometry_msgs::Vector3>("/wind_3d", 1, wind_commanded_cb);

    ros::Rate rate(1/sampleTime);

    ROS_INFO("Checking code for immediate rerun!");
    for (int i=0;i<(int)2/sampleTime;i++)
    {
        if (start_rec_cmd)
        {
            ROS_WARN("start_rec_cmd = 1: recording already started!");
            ROS_WARN("Restart recording to record data!");
            ROS_WARN("Please uncheck start_rec_cmd and exit (ctrl+c) the code!");
            while(ros::ok())
            {
                ros::spinOnce();
                rate.sleep();
            }
        }
        if (trajectory_start_flag.data)
        {
            ROS_WARN("traj_start = 1: trajectory already started!");
            ROS_WARN("Restart trajectory to record data!");
            ROS_WARN("Please uncheck traj_start and exit (ctrl+c) the code!");
            while(ros::ok())
            {
                ros::spinOnce();
                rate.sleep();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("No immediate rerun found: running code!");

    string path_dir = ros::package::getPath("dji_m100_trajectory") + "/Exp_data/";
    read_name.open(path_dir + "name_file.txt");
    if (!read_name)
    {
        ROS_ERROR("Unable to open file: name_file.txt!");
        ROS_ERROR("Exiting code!");
        exit(0);   // call system to stop
    }
    string line,file_name;
    while (std::getline(read_name, line))
        file_name += line;
    read_name.close();

/*
    print_results<<"time"<<","      // col 1
    // reference traj data: 3 cols: 2->4
                 <<"ref_pos_x_m"<<","<<"ref_pos_y_m"<<","<<"ref_pos_z_m"<<","
    // reference traj_delay data: 3 cols: 5->7
                 <<"ref_pos_delayed_x_m"<<","<<"ref_pos_delayed_y_m"<<","<<"ref_pos_delayed_z_m"<<","
    // reference vel data: 3 cols: 8->10
                 <<"ref_vel_u_m_sec"<<","<<"ref_vel_v_m_sec"<<","<<"ref_vel_w_m_sec"<<","
    // UAV feedback data: 12 cols: 11->22
                 <<"pos_x_m"<<","<<"pos_y_m"<<","<<"pos_z_m"<<","<<"vel_u_m_sec"<<","<<"vel_v_m_sec"<<","<<"vel_w_m_sec"<<","
                 <<"att_phi_deg"<<","<<"att_theta_deg"<<","<<"att_psi_deg"<<","<<"angvel_p_deg_sec"<<","<<"angvel_q_deg_sec"<<","<<"angvel_r_deg_sec"<<","
    // NMPC data: 6 cols: 23->28
                 <<"nmpc_roll_deg"<<","<<"nmpc_pitch_deg"<<","<<"nmpc_yaw_deg"<<","<<"nmpc_Fz_N"<<","<<"nmpc_exeTime"<<","<<"nmpc_kkt"<<",";
    // Wind command data: 3 cols: 29->31
                 <<"wind_x"<<","<<"wind_y"<<","<<"wind_z"<<","
    // Wind force estimation mean data at 0: 3 cols: 32->34
                 <<"dist_Fx_mu[0]"<<","<<"dist_Fy_mu[0]"<<","<<"dist_Fz_mu[0]"<<<","
    if rec_config == GP
        // Wind force estimation mean data at pred_horizon: 3 cols: 35->37
                     <<"dist_Fx_mu[pred_horizon]"<<","<<"dist_Fy_mu[pred_horizon]"<<","<<"dist_Fz_mu[pred_horizon]"<<<","
        // Wind force estimation variance data at 0: 3 cols: 38->40
                     <<"dist_Fx_var[0]"<<","<<"dist_Fy_var[0]"<<","<<"dist_Fz_var[0]"<<<","
        // Wind force estimation variance data at pred_horizon: 3 cols: 41->43
                     <<"dist_Fx_var[pred_horizon]"<<","<<"dist_Fy_var[pred_horizon]"<<","<<"dist_Fz_var[pred_horizon]"<< endl;
    if rec_config == NMHE
        // NMHE velocity estimation: 3 cols: 35->37
                     <<"nmhe_vel_u_m_sec"<<","<<"nmhe_vel_v_m_sec"<<","<<"nmhe_vel_w_m_sec"<<","
        // NMHE data: 2 cols: 38->39
                     <<"nmpc_exeTime"<<","<<"nmpc_kkt"<< endl;
    ROS_INFO("Header written!");
*/

    current_pos_att = {0, 0, 0, 0, 0, 0};
    current_vel_rate = {0, 0, 0, 0, 0, 0};
    dist_Fx_mu.resize(pred_horizon + 1);
    dist_Fy_mu.resize(pred_horizon + 1);
    dist_Fz_mu.resize(pred_horizon + 1);
    dist_Fx_var.resize(pred_horizon + 1);
    dist_Fy_var.resize(pred_horizon + 1);
    dist_Fz_var.resize(pred_horizon + 1);

    while(ros::ok())
    {
        if (start_rec_cmd)
        {
            if (print_start_rec_cmd_flag == 1)
            {
                std::cout<<"start_rec --> 1\n";

                switch (rec_config) {
                case 0:
                    dist_Fx_mu_sub = nh.subscribe<std_msgs::Float64MultiArray>("gp_disturb_reg/mu/x", 1, dist_Fx_mu_cb);
                    dist_Fy_mu_sub = nh.subscribe<std_msgs::Float64MultiArray>("gp_disturb_reg/mu/y", 1, dist_Fy_mu_cb);
                    dist_Fz_mu_sub = nh.subscribe<std_msgs::Float64MultiArray>("gp_disturb_reg/mu/z", 1, dist_Fz_mu_cb);
                    dist_Fx_var_sub = nh.subscribe<std_msgs::Float64MultiArray>("gp_disturb_reg/var/x", 1, dist_Fx_var_cb);
                    dist_Fy_var_sub = nh.subscribe<std_msgs::Float64MultiArray>("gp_disturb_reg/var/y", 1, dist_Fy_var_cb);
                    dist_Fz_var_sub = nh.subscribe<std_msgs::Float64MultiArray>("gp_disturb_reg/var/z", 1, dist_Fz_var_cb);
                    std::cout<<"record config = GP!\n";
                    break;
                case 1:
                    dist_Fx_mu_sub = nh.subscribe<std_msgs::Float64MultiArray>("nmhe_learning/Fx", 1, dist_Fx_mu_cb);
                    dist_Fy_mu_sub = nh.subscribe<std_msgs::Float64MultiArray>("nmhe_learning/Fy", 1, dist_Fy_mu_cb);
                    dist_Fz_mu_sub = nh.subscribe<std_msgs::Float64MultiArray>("nmhe_learning/Fz", 1, dist_Fz_mu_cb);
                    nmhe_vel_sub = nh.subscribe<std_msgs::Float64MultiArray>("nmhe_learning/uvw", 1, nmhe_uvw_cb);
                    nmhe_exeTime_sub = nh.subscribe<std_msgs::Float64>("nmhe_learning/exeTime", 1, nmpc_exeTime_cb);
                    nmhe_kkt_sub = nh.subscribe<std_msgs::Float64>("nmhe_learning/kkt", 1, nmpc_kkt_cb);
                    std::cout<<"record config = NMHE!\n";
                }

                print_start_rec_cmd_flag = 0;
            }
        }
        else
        {
            if (print_start_rec_cmd_flag == 0)
            {
                std::cout<<"start_rec --> 0\n";
                print_start_rec_cmd_flag = 1;
            }
        }

        t = ros::Time::now().toSec();
        t_last = t;

        while(ros::ok() && start_rec_cmd && trajectory_start_flag.data)
        {
            if(print_rec_start_flag == 1)
            {
                std::cout<<"---------------------------------\n";
//                file_name = "real_" + file_name;
                print_results.open(path_dir + file_name);
                std::cout<<"File created with name: "<<file_name.c_str()<<"\n";
                std::cout<<"Recording started!\n";

                rec_started_flag = true;
                print_rec_start_flag = 0;
            }

            t = ros::Time::now().toSec();
            traj_time = t - t_last;

            print_results<< std::fixed << std::setprecision(6) <<traj_time<<",";
            for(int j=0; j<3; j++)
                print_results<<ref_trajectory(j)<<",";
         //   for(int j=0; j<3; j++)
         //       print_results<<ref_trajectory_delay(j)<<",";
            for(int j=0; j<3; j++)
                print_results<<current_pos(j)<<",";
            for(int j=0; j<3; j++)
                print_results<<point(j)<<","; 
            for(int j=0; j<3; j++)
                print_results<<normal(j)<<",";       
            for(int j=0; j<3; j++)
                print_results<<current_vel(j)<<",";
            for(int j=0; j<3; j++)
                print_results<<current_att(j)<<",";
            for(int j=0; j<3; j++)
                print_results<<current_vel(j)<<",";
            for(int j=0; j<3; j++)
                print_results<<ref_velocity(j)<<",";  

            print_results<<nmpc_obj<<",";
            print_results<<nmpc_kkt<<",";

            for(int j=0; j<3; j++)
                print_results<<current_rates(j)<<",";
            print_results<<rad2deg*nmpc_ryp(0)<<","<<rad2deg*nmpc_ryp(1)<<","<<rad2deg*nmpc_ryp(2)
                         <<","<<nmpc_Fz(0)<<","<<nmpc_exeTime<<","<<nmpc_kkt<<",";
            for(int j=0; j<3; j++)
            {
                if (j<2)
                    print_results<<wind_commanded(j)<<",";
                else
                    print_results<<wind_commanded(j);
            }
            if (rec_Fx_dist)
                print_results<<","<<dist_Fx_mu[0];
            if (rec_Fy_dist)
                print_results<<","<<dist_Fy_mu[0];
            if (rec_Fz_dist)
                print_results<<","<<dist_Fz_mu[0];
            switch (rec_config) {
            case 0:
                if (rec_Fx_dist)
                    print_results<<","<<dist_Fx_mu[pred_horizon];
                if (rec_Fy_dist)
                    print_results<<","<<dist_Fy_mu[pred_horizon];
                if (rec_Fz_dist)
                    print_results<<","<<dist_Fz_mu[pred_horizon];
                if (rec_Fx_dist)
                    print_results<<","<<dist_Fx_var[0];
                if (rec_Fy_dist)
                    print_results<<","<<dist_Fy_var[0];
                if (rec_Fz_dist)
                    print_results<<","<<dist_Fz_var[0];
                if (rec_Fx_dist)
                    print_results<<","<<dist_Fx_var[pred_horizon];
                if (rec_Fy_dist)
                    print_results<<","<<dist_Fy_var[pred_horizon];
                if (rec_Fz_dist)
                    print_results<<","<<dist_Fz_var[pred_horizon];
                break;
            case 1:
                for(int j=0; j<3; j++)
                    print_results<<","<<nmhe_uvw(j);
                print_results<<","<<nmpc_exeTime<<","<<nmpc_kkt;
            }
            print_results<<endl;

            if (std::fmod(std::abs(traj_time - std::floor(traj_time)), (double)(1)) < sampleTime)
                std::cout<<"Record time: " << (int)traj_time << " (sec)"<<"\n";

            if (traj_time >= max_rec_time || stop_rec_cmd)
            {
                std::cout<<"---------------------------------\n";
                std::cout<<"Recording complete at trajectory time: "<<traj_time<<" sec!\n";
//                ROS_INFO("Exiting code!");
                std::cout<<"Please exit (ctrl+c) the code and start new trajectory before next recording!\n";
                print_results.close();
                rec_started_flag = false;
                while(ros::ok())
                {
                    ros::spinOnce();
                    rate.sleep();
                }
            }

            ros::spinOnce();
            rate.sleep();
        }

        ros::spinOnce();
        rate.sleep();

        if(print_rec_start_flag == 0 && rec_started_flag)
        {
            std::cout<<"---------------------------------\n";
            std::cout<<"Recording stopped at trajectory time: "<<traj_time<<" sec!\n";
            std::cout<<"Please exit (ctrl+c) the code and start new trajectory before next recording!\n";
            print_results.close();
            print_rec_start_flag = 1;
            rec_started_flag = false;
            while(ros::ok())
            {
                ros::spinOnce();
                rate.sleep();
            }
        }

        if(stop_rec_cmd)
        {
            std::cout<<"---------------------------------\n";
            std::cout<<"Recording stopped at trajectory time: "<<traj_time<<" sec!\n";
            std::cout<<"Please uncheck stop_rec before exiting (ctrl+c) the code!\n";
            print_results.close();
            while(ros::ok())
            {
                ros::spinOnce();
                rate.sleep();
            }
        }

    }

    return 0;

}
