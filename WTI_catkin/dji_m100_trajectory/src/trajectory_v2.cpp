/**
 * @file   trajectory_v2.cpp
 * @author Mohit Mehndiratta
 * @date   April 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <trajectory_v2.h>
#include <dji_m100_trajectory/set_trajectory_v2Config.h>
std_msgs::Float64 px_test,py_test,nx_test,ny_test;

double sampleTime = 0.01;

void dynamicReconfigureCallback(dji_m100_trajectory::set_trajectory_v2Config& config, uint32_t level)
{
    traj_on = config.traj_on;
    max_z_on = config.max_z_on;
    lidar_on = config.lidar_on;
    adaptive_yaw_on = config.adaptive_yaw_on;
    reg_on = config.reg_on;
    climb_flag = config.climb;
    land_flag = config.land;
    change_z = config.change_z;
    point_tracking_on = config.point_tracking_on;
    pub_setpoint_pos = config.pub_on_setpoint_position;

    traj_type = config.traj_type;
    pos_pub_delay = config.pos_pub_delay;
    max_z = config.max_z;
    x_hover = config.x_hover;
    v_d = config.v_d;
    px = config.px;
    py = config.py;
    pz = config.pz;

    nxx = config.nx;
    nyy = config.ny;
    nzz = config.nz;

    y_hover = config.y_hover;
    z_hover = config.z_hover;
    yaw_hover = config.yaw_hover;
    wall_dist = config.wall_dist;
    x_sp_start = config.x_sp_start;
    y_sp_start = config.y_sp_start;
    z_sp_start = config.z_sp_start;
    length = config.length;
    height = config.height;
    camera_V_FOV = config.camera_V_FOV;
    req_V_overlap_percent = config.req_V_overlap_percent;
    del_z = config.del_z;
    radius = config.des_radius;
    absvel = config.des_velocity;

    rotvel = absvel / radius;
    time_period = 2 * M_PI / rotvel;

    climb_rate = config.climb_rate;
    land_rate = config.land_rate;
}

// Callback function
std::vector<double> current_pos, current_att(3, 0.0);
geometry_msgs::PoseStamped desired_pos;
geometry_msgs::PoseStamped point;
geometry_msgs::PoseStamped point_delayed;
geometry_msgs::PoseStamped normal;
geometry_msgs::PoseStamped vel_d_g;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    // TO BE: try to get the Euler angles in one line of code!
    current_att_quat = {
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
    current_att_mat.setRotation(current_att_quat);
    current_att_mat.getRPY(current_att[0], current_att[1], current_att[2]);
}

void GP_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    desired_pos = *msg;
}


void vel_point_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   vel_d_g = *msg;

}


void point_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    point = *msg;
}



void px_test_cb(const std_msgs::Float64::ConstPtr& msg)
{
    px_test = *msg;
}


void py_test_cb(const std_msgs::Float64::ConstPtr& msg)
{
    py_test = *msg;
}


void nx_test_cb(const std_msgs::Float64::ConstPtr& msg)
{
    nx_test = *msg;
}

void ny_test_cb(const std_msgs::Float64::ConstPtr& msg)
{
    ny_test = *msg;
}







void point_cb_delayed(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    point_delayed = *msg;
}

void norm_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    normal = *msg;
}

void Llidar_read_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if (!std::isnan(msg->range) || !std::isnan(Llidar_read_data_unfiltered[Llidar_read_data_unfiltered.size() - 1]))
    {
        // erase from the front!
        Llidar_read_data_unfiltered.erase(Llidar_read_data_unfiltered.begin());
        if (msg->range < msg->min_range)
            Llidar_read_data_unfiltered.push_back(Llidar_read_data_unfiltered.back());
        else if (msg->range > msg->max_range)
            Llidar_read_data_unfiltered.push_back(Llidar_read_data_unfiltered.back());
        else
            Llidar_read_data_unfiltered.push_back(msg->range);

        Llidar_read_data = 0.0;
        for (int i = 0; i < Llidar_read_data_unfiltered.size(); ++i)
            Llidar_read_data += Llidar_read_data_unfiltered[i];
        Llidar_read_data = Llidar_read_data / (Llidar_read_data_unfiltered.size());
    }
}

void Clidar_read_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if (!std::isnan(msg->range) || !std::isnan(Clidar_read_data_unfiltered[Clidar_read_data_unfiltered.size() - 1]))
    {
        // erase from the front!
        Clidar_read_data_unfiltered.erase(Clidar_read_data_unfiltered.begin());
        if (msg->range < msg->min_range)
            Clidar_read_data_unfiltered.push_back(Clidar_read_data_unfiltered.back());
        else if (msg->range > msg->max_range)
            Clidar_read_data_unfiltered.push_back(Clidar_read_data_unfiltered.back());
        else
            Clidar_read_data_unfiltered.push_back(msg->range);

        Clidar_read_data = 0.0;
        for (int i = 0; i < Clidar_read_data_unfiltered.size(); ++i)
            Clidar_read_data += Clidar_read_data_unfiltered[i];
        Clidar_read_data = Clidar_read_data / (Clidar_read_data_unfiltered.size());
    }
}

void Rlidar_read_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if (!std::isnan(msg->range) || !std::isnan(Rlidar_read_data_unfiltered[Rlidar_read_data_unfiltered.size() - 1]))
    {
        // erase from the front!
        Rlidar_read_data_unfiltered.erase(Rlidar_read_data_unfiltered.begin());
        if (msg->range < msg->min_range)
            Rlidar_read_data_unfiltered.push_back(Rlidar_read_data_unfiltered.back());
        else if (msg->range > msg->max_range)
            Rlidar_read_data_unfiltered.push_back(Rlidar_read_data_unfiltered.back());
        else
            Rlidar_read_data_unfiltered.push_back(msg->range);

        Rlidar_read_data = 0.0;
        for (int i = 0; i < Rlidar_read_data_unfiltered.size(); ++i)
            Rlidar_read_data += Rlidar_read_data_unfiltered[i];
        Rlidar_read_data = Rlidar_read_data / (Rlidar_read_data_unfiltered.size());
    }
}

void sonar_read_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if (!std::isnan(msg->range) || !std::isnan(sonar_read_data_unfiltered[sonar_read_data_unfiltered.size() - 1]))
    {
        // erase from the front!
        sonar_read_data_unfiltered.erase(sonar_read_data_unfiltered.begin());
        if (msg->range < msg->min_range)
            sonar_read_data_unfiltered.push_back(sonar_read_data_unfiltered.back());
        else if (msg->range > msg->max_range)
            sonar_read_data_unfiltered.push_back(sonar_read_data_unfiltered.back());
        else
            sonar_read_data_unfiltered.push_back(msg->range);

        sonar_read_data = 0.0;
        for (int i = 0; i < sonar_read_data_unfiltered.size(); ++i)
            sonar_read_data += sonar_read_data_unfiltered[i];
        sonar_read_data = sonar_read_data / (sonar_read_data_unfiltered.size());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "m100_trajectory");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<dji_m100_trajectory::set_trajectory_v2Config> server;
    dynamic_reconfigure::Server<dji_m100_trajectory::set_trajectory_v2Config>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    // Roslaunch parameters
    ros::param::get("mocap_topic", mocap_topic);
    ros::param::get("Llidar_topic", Llidar_topic);
    ros::param::get("Clidar_topic", Clidar_topic);
    ros::param::get("Rlidar_topic", Rlidar_topic);
    ros::param::get("use_sonar", use_sonar);
    ros::param::get("sonar_topic", sonar_topic);
    ros::param::get("use_current_pos", use_current_pos);

    // Publisher
    ref_pos_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/position", 1);
    ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ref_trajectory/pose", 1);
    ref_pos_delay_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/position_delayed", 1);
    ref_vel_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/velocity", 1);
    ref_yaw_pub = nh.advertise<std_msgs::Float64>("ref_trajectory/yaw", 1);
    setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    traj_on_pub = nh.advertise<std_msgs::Bool>("trajectory_on", 1);
    reg_on_pub = nh.advertise<std_msgs::Bool>("regression_on", 1);

    ros::Publisher lidar_read_filtered_pub = nh.advertise<std_msgs::Float64>("range_filter", 1);
    ros::Publisher drone_velocity_pub = nh.advertise<std_msgs::Float64>("drone_vel", 1);
    point_to_view_pub = nh.advertise<geometry_msgs::PoseStamped>("/point_to_view", 1);


    norm_desired_pub = nh.advertise<geometry_msgs::PoseStamped>("/surface_normal", 1);

    // Subscriber
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(mocap_topic, 1, pos_cb);
    Llidar_read_sub = nh.subscribe<sensor_msgs::Range>(Llidar_topic, 1, Llidar_read_cb);
    Clidar_read_sub = nh.subscribe<sensor_msgs::Range>(Clidar_topic, 1, Clidar_read_cb);
    Rlidar_read_sub = nh.subscribe<sensor_msgs::Range>(Rlidar_topic, 1, Rlidar_read_cb);
    ros::Subscriber GP_WP_sub = nh.subscribe<geometry_msgs::PoseStamped>("/WP_GP", 1, GP_cb);
    ros::Subscriber velocity_ref_sub = nh.subscribe<geometry_msgs::PoseStamped>("/point_vel_ref", 1, vel_point_cb);

    ros::Subscriber point_sub = nh.subscribe<geometry_msgs::PoseStamped>("/point_to_view_traj", 1, point_cb);
    ros::Subscriber point_delayed_sub = nh.subscribe<geometry_msgs::PoseStamped>("/point_to_view_traj_delayed", 1, point_cb_delayed);
    ros::Subscriber norm_sub = nh.subscribe<geometry_msgs::PoseStamped>("/norm_traj", 1, norm_cb);


   // test
    ros::Subscriber px_test_sub = nh.subscribe<std_msgs::Float64>("px", 1, px_test_cb);
    ros::Subscriber py_test_sub = nh.subscribe<std_msgs::Float64>("py", 1, py_test_cb);
    ros::Subscriber nx_test_sub = nh.subscribe<std_msgs::Float64>("nx", 1, nx_test_cb);
    ros::Subscriber ny_test_sub = nh.subscribe<std_msgs::Float64>("ny", 1, ny_test_cb);
    if (use_sonar)
        sonar_read_sub = nh.subscribe<sensor_msgs::Range>(sonar_topic, 1, sonar_read_cb);

    ros::Rate rate(1 / sampleTime);

    pos_ref_start_msg.pose.position.x = 0;
    pos_ref_start_msg.pose.position.y = 0;
    if (max_z_on)
        pos_ref_start_msg.pose.position.z = max_z;
    else
        pos_ref_start_msg.pose.position.z = 0;

    //    traj_on_msg.data = false;

    while (ros::ok())
    {
        traj_on_msg.data = traj_on;
        reg_on_msg.data = traj_on ? reg_on : false;

        if (max_z_on && pos_ref_start_msg.pose.position.z != max_z && !landed_flag && traj_on != 1)
        {
            pos_ref_start_msg.pose.position.z = max_z;
            print_flag_traj_on = 0;
        }
        else if (!max_z_on && pos_ref_start_msg.pose.position.z != 0.0 && traj_on != 1)
        {
            pos_ref_start_msg.pose.position.z = 0.0;
            print_flag_traj_on = 0;
        }

        if (change_z == 1)
        {
            if (print_flag_changez == 1)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Changing z by %.2f m!", del_z);
                print_flag_changez = 2;
            }
            const_z = 1;
        }
        else
        {
            if (print_flag_changez == 2)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Constant z!");
                print_flag_changez = 1;
            }
            const_z = 0;
        }

        if (traj_on == 1 && !landed_flag)
        {
            //            traj_on_msg.data = traj_on;
            if (!traj_started_flag)
                traj_started_flag = true;

            t = ros::Time::now().toSec();
            traj_time = t - t_last;

            if (climb_flag)
            {
                while (ros::ok() && !climbed_flag && z_delay < max_z)
                {
                    if (print_flag_climb == 0)
                    {
                        ROS_INFO("---------------------------------");
                        ROS_INFO("Climbing initialized!");
                        print_flag_climb = 1;

                        z_delay_start = z;
                    }
                    x = x;
                    y = y;
                    z = z < max_z ? z + climb_rate * sampleTime : max_z;

                    x_delay = x;
                    y_delay = y;
                    z_delay = std::abs(z - z_delay_start) < climb_rate * pos_pub_delay * sampleTime
                                  ? z_delay_start
                                  : z_delay + climb_rate * sampleTime;

                    publish_inspection_point();
                   // double u_global = ( point.pose.position.x -  point_delayed.pose.position.x) / sampleTime;
                   // double v_global = (point.pose.position.y -  point_delayed.pose.position.y) / sampleTime;
                   // double w_global = ( point.pose.position.z -  point_delayed.pose.position.z) / sampleTime;
                    double u_global = vel_d_g.pose.position.x;
                    double v_global = vel_d_g.pose.position.y;
                    double w_global = vel_d_g.pose.position.z;


                    // desired velocity data in body frame
                    tf::Matrix3x3 rotational_matrix_BI(current_att_quat);
                    rotational_matrix_BI = rotational_matrix_BI.transpose();

                    u = rotational_matrix_BI[0][0] * u_global + rotational_matrix_BI[0][1] * v_global +
                        rotational_matrix_BI[0][2] * w_global;
                    v = rotational_matrix_BI[1][0] * u_global + rotational_matrix_BI[1][1] * v_global +
                        rotational_matrix_BI[1][2] * w_global;
                    w = rotational_matrix_BI[2][0] * u_global + rotational_matrix_BI[2][1] * v_global +
                        rotational_matrix_BI[2][2] * w_global;


                   

                    ref_yaw_msg.data = compute_ref_yaw();
                    if (!pub_setpoint_pos)
                    {
                        reftrajectory_msg.x = x;
                        reftrajectory_msg.y = y;
                        reftrajectory_msg.z = z;
                        ref_pos_pub.publish(reftrajectory_msg);
                        reftrajectory_delay_msg.x = x_delay;
                        reftrajectory_delay_msg.y = y_delay;
                        reftrajectory_delay_msg.z = z_delay;
                        ref_pos_delay_pub.publish(reftrajectory_delay_msg);

                        //reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
                        //reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
                        //reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);

                        




                        ref_vel_pub.publish(reftrajectory_vel_msg);

                        ref_yaw_pub.publish(ref_yaw_msg);

                        // For rviz visualization:
                        setpoint_pos_msg.header.stamp = ros::Time::now();
                        setpoint_pos_msg.header.frame_id = "map";
                        setpoint_pos_msg.pose.position.x = x;
                        setpoint_pos_msg.pose.position.y = y;
                        setpoint_pos_msg.pose.position.z = z;

                        setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);
                        setpoint_pos_msg.pose.orientation.x = desired_pos.pose.orientation.x;
                        setpoint_pos_msg.pose.orientation.y = desired_pos.pose.orientation.y;
                        setpoint_pos_msg.pose.orientation.z = desired_pos.pose.orientation.z;
                        setpoint_pos_msg.pose.orientation.w = desired_pos.pose.orientation.w;
                        ref_pose_pub.publish(setpoint_pos_msg);
                    }
                    else
                    {
                        setpoint_pos_msg.header.stamp = ros::Time::now();
                        setpoint_pos_msg.header.frame_id = "map";
                        setpoint_pos_msg.pose.position.x = x;
                        setpoint_pos_msg.pose.position.y = y;
                        setpoint_pos_msg.pose.position.z = z;

                        setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);
                        setpoint_pos_msg.pose.orientation.x = desired_pos.pose.orientation.x;
                        setpoint_pos_msg.pose.orientation.y = desired_pos.pose.orientation.y;
                        setpoint_pos_msg.pose.orientation.z = desired_pos.pose.orientation.z;
                        setpoint_pos_msg.pose.orientation.w = desired_pos.pose.orientation.w;
                        setpoint_pos_pub.publish(setpoint_pos_msg);
                        ref_pose_pub.publish(setpoint_pos_msg);
                    }

                    if (z_delay >= max_z)
                    {
                        if (print_flag_climb == 1)
                        {
                            ROS_INFO("Climbing complete!");
                            print_flag_climb = 0;
                        }
                        pos_ref_start_msg.pose.position.z = max_z;
                        climbed_flag = true;
                        landed_flag = false;
                        t_last = ros::Time::now().toSec();
                    }

                    x_last = x;
                    y_last = y;
                    z_last = z;

                    traj_on_pub.publish(traj_on_msg);
                    reg_on_pub.publish(reg_on_msg);
                    ros::spinOnce();
                    rate.sleep();
                }
            }

            if (climbed_flag && !landed_flag)
            {
                pos_ref_start_msg.pose.position.z = max_z;
            }

            if (print_flag_traj_on == 1)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Reference trajectory started!");
                print_flag_traj_on = 0;
            }
            if (traj_type == 0)  // hover at origin
            {
                if (print_flag_hover_origin == 1)
                {
                    ROS_INFO("--------Hover at origin selected!--------");
                    print_flag_hover_origin = 0;
                    print_flag_hover = 1;
                    print_flag_hover_lidar = 1;
                    print_flag_circle = 1;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;
                    print_flag_setpoint = 1;
                    print_flag_GP = 1;
                }
                x = pos_ref_start_msg.pose.position.x;
                y = pos_ref_start_msg.pose.position.y;
                //                z = pos_ref_start_msg.pose.position.z - del_z*(sin(traj_time*3/7)) * const_z;
                z = pos_ref_start_msg.pose.position.z;
                x_delay = x;
                y_delay = y;
                z_delay = z;

                t_last = ros::Time::now().toSec();
            }

            if (traj_type == 1)  // hover
            {
                if (print_flag_hover == 1)
                {
                    t_last = ros::Time::now().toSec();

                    // Quick fix to hover at the current location when switched from setpoint trajectory
                    // TO BE: check if this condition can be used for all the trajectories?
                    if (print_flag_setpoint == 0)
                    {
                        x_atTrajStart = x - x_hover;
                        y_atTrajStart = y - y_hover;
                        z_atTrajStart = z - z_hover;
                    }
                    else
                    {
                        x_atTrajStart = x;
                        y_atTrajStart = y;
                        z_atTrajStart = z;
                    }

                    ROS_INFO("--------Hover selected!--------");
                    print_flag_hover_origin = 1;
                    print_flag_hover = 0;
                    print_flag_hover_lidar = 1;
                    print_flag_circle = 1;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;
                    print_flag_setpoint = 1;
                    print_flag_GP = 1;

                    // TO BE: check is required?
                    x_B_atTrajStart = R_BI[0][0] * x_atTrajStart + R_BI[0][1] * y_atTrajStart;
                    y_B_atTrajStart = R_BI[1][0] * x_atTrajStart + R_BI[1][1] * y_atTrajStart;
                    x_B = x_B_atTrajStart;
                    y_B = y_B_atTrajStart;

                    z_delay_started = false;
                    z_delay_start = z;
                }
                x = x_atTrajStart + x_hover;
                y = y_atTrajStart + y_hover;
                z = z_atTrajStart + z_hover - del_z * (sin(rotvel * traj_time)) * const_z;
                x_delay = x;
                y_delay = y;
                if (!z_delay_started)
                    if (std::abs(z - z_delay_start) < del_z * (sin(rotvel * (pos_pub_delay * sampleTime))) * const_z)
                        z_delay = z_delay_start;
                    else
                    {
                        z_delay = z_atTrajStart + z_hover -
                                  del_z * (sin(rotvel * (traj_time - pos_pub_delay * sampleTime))) * const_z;
                        //                        std::cout<<"z_delay_started gets true"<<"\n";
                        z_delay_started = true;
                    }
                else
                    z_delay = z_atTrajStart + z_hover -
                              del_z * (sin(rotvel * (traj_time - pos_pub_delay * sampleTime))) * const_z;
            }

            if (traj_type == 2)  // hover_lidar
            {
                if (print_flag_hover_lidar == 1)
                {
                    t_last = ros::Time::now().toSec();

                    ROS_INFO("--------Hover with lidar selected!--------");
                    print_flag_hover_origin = 1;
                    print_flag_hover = 1;
                    print_flag_hover_lidar = 0;
                    print_flag_circle = 1;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;
                    print_flag_setpoint = 1;
                    print_flag_GP = 1;
                    x_atTrajStart = x;
                    y_atTrajStart = y;
                    z_atTrajStart = z;

                    setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);
                    R_BI.setRotation(setpoint_att_quat);
                    R_BI = R_BI.transpose();
                    R_IB = R_BI.transpose();

                    std::cout << "R_BI = ";
                    for (int i = 0; i < 3; i++)
                    {
                        for (int j = 0; j < 3; j++)
                            std::cout << R_BI[i][j] << ", ";
                        std::cout << "\n";
                    }
                    std::cout << "R_IB = ";
                    for (int i = 0; i < 3; i++)
                    {
                        for (int j = 0; j < 3; j++)
                            std::cout << R_IB[i][j] << ", ";
                        std::cout << "\n";
                    }

                    x_B_atTrajStart = R_BI[0][0] * x_atTrajStart + R_BI[0][1] * y_atTrajStart;
                    y_B_atTrajStart = R_BI[1][0] * x_atTrajStart + R_BI[1][1] * y_atTrajStart;
                    x_B = x_B_atTrajStart;
                    y_B = y_B_atTrajStart;

                    z_delay_started = false;
                    z_delay_start = z;

                    // TO BE: removed
                    if (!use_current_pos)
                    {
                        ROS_WARN("use_current_pos is set to false!");
                        ROS_WARN("Not utilizing position feedback!");
                    }
                }
                if (lidar_on)
                {
                    //                    x_B = x_B_atTrajStart + (compute_actual_wall_dist()()-wall_dist);
                    //                    y_B = y_B_atTrajStart + y_hover + (y_B_atTrajStart -
                    //                                                       (R_BI[1][0]*current_pos[0] +
                    //                                                        R_BI[1][1]*current_pos[1]));
                    //                    x = R_IB[0][0]*x_B + R_IB[0][1]*y_B + current_pos[0];
                    //                    y = R_IB[1][0]*x_B + R_IB[1][1]*y_B + current_pos[1];

                    // TO BE: removed
                    if (use_current_pos)
                        x_B = x_B_atTrajStart + (compute_actual_wall_dist() - wall_dist) +
                              (R_BI[0][0] * current_pos[0] + R_BI[0][1] * current_pos[1]);
                    else
                        x_B = x_B_atTrajStart + (compute_actual_wall_dist() - wall_dist);
                    y_B = y_B_atTrajStart + y_hover;
                    x = R_IB[0][0] * x_B + R_IB[0][1] * y_B;
                    y = R_IB[1][0] * x_B + R_IB[1][1] * y_B;
                }
                else
                {
                    x = x_atTrajStart + x_hover;
                    y = y_atTrajStart + y_hover;
                }
                z = z_atTrajStart + z_hover - del_z * (sin(rotvel * traj_time)) * const_z;
                x_delay = x;
                y_delay = y;
                if (!z_delay_started)
                    if (std::abs(z - z_delay_start) < del_z * (sin(rotvel * (pos_pub_delay * sampleTime))) * const_z)
                        z_delay = z_delay_start;
                    else
                    {
                        z_delay = z_atTrajStart + z_hover -
                                  del_z * (sin(rotvel * (traj_time - pos_pub_delay * sampleTime))) * const_z;
                        //                        std::cout<<"z_delay_started gets true"<<"\n";
                        z_delay_started = true;
                    }
                else
                    z_delay = z_atTrajStart + z_hover -
                              del_z * (sin(rotvel * (traj_time - pos_pub_delay * sampleTime))) * const_z;
            }

            if (traj_type == 3)  // circle
            {
                if (print_flag_circle == 1)
                {
                    t_last = ros::Time::now().toSec();

                    ROS_INFO("--------Circle selected!--------");
                    ROS_INFO("rotational velocity = %.2f rad/s", rotvel);
                    ROS_INFO("time period = %.2f s", time_period);
                    print_flag_hover_origin = 1;
                    print_flag_hover = 1;
                    print_flag_hover_lidar = 1;
                    print_flag_circle = 0;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;
                    print_flag_setpoint = 1;
                    print_flag_GP = 1;
                    x_atTrajStart = pos_ref_start_msg.pose.position.x + x_hover;
                    y_atTrajStart = pos_ref_start_msg.pose.position.y + y_hover;
                    z_atTrajStart = pos_ref_start_msg.pose.position.z + z_hover;

                    x_delay_started = false;
                    y_delay_started = false;
                    z_delay_started = false;
                    x_delay_start = x;
                    y_delay_start = y;
                    z_delay_start = z;
                }
                x = x_atTrajStart + radius * sin(rotvel * traj_time);
                if (!x_delay_started)
                    if (std::abs(x - x_delay_start) < radius * (sin(rotvel * (pos_pub_delay * sampleTime))))
                        x_delay = x_delay_start;
                    else
                    {
                        x_delay = x_atTrajStart + radius * sin(rotvel * (traj_time - pos_pub_delay * sampleTime));
                        //                        std::cout<<"x_delay_started gets true"<<"\n";
                        x_delay_started = true;
                    }
                else
                    x_delay = x_atTrajStart + radius * sin(rotvel * (traj_time - pos_pub_delay * sampleTime));

                if (traj_time < 0.25 * time_period)
                {
                    y = y_atTrajStart;
                    z = z_atTrajStart;
                    y_delay = y;
                    z_delay = z;
                    t_last_z = ros::Time::now().toSec();
                }
                else
                {
                    traj_time_z = t - t_last_z;
                    y = y_atTrajStart + radius * cos(rotvel * traj_time);
                    z = z_atTrajStart - del_z * ((sin(rotvel * traj_time_z))) * const_z;

                    if (!y_delay_started)
                        if (std::abs(y - y_delay_start) < radius * (sin(rotvel * (pos_pub_delay * sampleTime))))
                            y_delay = y_delay_start;
                        else
                        {
                            y_delay = y_atTrajStart + radius * cos(rotvel * (traj_time - pos_pub_delay * sampleTime));
                            //                            std::cout<<"y_delay_started gets true"<<"\n";
                            y_delay_started = true;
                        }
                    else
                        y_delay = y_atTrajStart + radius * cos(rotvel * (traj_time - pos_pub_delay * sampleTime));

                    if (!z_delay_started)
                        if (std::abs(z - z_delay_start) <
                            del_z * (sin(rotvel * (pos_pub_delay * sampleTime))) * const_z)
                            z_delay = z_delay_start;
                        else
                        {
                            z_delay = z_atTrajStart -
                                      del_z * (sin(rotvel * (traj_time_z - pos_pub_delay * sampleTime))) * const_z;
                            //                            std::cout<<"z_delay_started gets true"<<"\n";
                            z_delay_started = true;
                        }
                    else
                        z_delay = z_atTrajStart -
                                  del_z * (sin(rotvel * (traj_time_z - pos_pub_delay * sampleTime))) * const_z;
                }
            }

            if (traj_type == 4)  // figure 8
            {
                if (print_flag_fig8 == 1)
                {
                    t_last = ros::Time::now().toSec();

                    ROS_INFO("--------Figure8 selected!--------");
                    print_flag_hover_origin = 1;
                    print_flag_hover = 1;
                    print_flag_hover_lidar = 1;
                    print_flag_circle = 1;
                    print_flag_fig8 = 0;
                    print_flag_square = 1;
                    print_flag_setpoint = 1;
                    print_flag_GP = 1;
                    x_atTrajStart = pos_ref_start_msg.pose.position.x + x_hover;
                    y_atTrajStart = pos_ref_start_msg.pose.position.y + y_hover;
                    z_atTrajStart = pos_ref_start_msg.pose.position.z + z_hover;

                    x_delay_started = false;
                    y_delay_started = false;
                    z_delay_started = false;
                    x_delay_start = x;
                    y_delay_start = y;
                    z_delay_start = z;
                }
                x = x_atTrajStart - radius * cos(rotvel * traj_time + M_PI / 2);
                if (!x_delay_started)
                    if (std::abs(x - x_delay_start) < radius * (sin(rotvel * (pos_pub_delay * sampleTime))))
                        x_delay = x_delay_start;
                    else
                    {
                        x_delay =
                            x_atTrajStart - radius * cos(rotvel * (traj_time - pos_pub_delay * sampleTime) + M_PI / 2);
                        //                        std::cout<<"x_delay_started gets true"<<"\n";
                        x_delay_started = true;
                    }
                else
                    x_delay =
                        x_atTrajStart - radius * cos(rotvel * (traj_time - pos_pub_delay * sampleTime) + M_PI / 2);

                y = y_atTrajStart + (radius / 2) * sin(2 * rotvel * traj_time);
                if (!y_delay_started)
                    if (std::abs(y - y_delay_start) < (radius / 2) * (sin(2 * rotvel * (pos_pub_delay * sampleTime))))
                        y_delay = y_delay_start;
                    else
                    {
                        y_delay =
                            y_atTrajStart + (radius / 2) * sin(2 * rotvel * (traj_time - pos_pub_delay * sampleTime));
                        //                        std::cout<<"y_delay_started gets true"<<"\n";
                        y_delay_started = true;
                    }
                else
                    y_delay = y_atTrajStart + (radius / 2) * sin(2 * rotvel * (traj_time - pos_pub_delay * sampleTime));

                z = z_atTrajStart + del_z * (sin(rotvel * traj_time)) * const_z;
                if (!z_delay_started)
                    if (std::abs(z - z_delay_start) < del_z * (sin(rotvel * (pos_pub_delay * sampleTime))) * const_z)
                        z_delay = z_delay_start;
                    else
                    {
                        z_delay =
                            z_atTrajStart + del_z * (sin(rotvel * (traj_time - pos_pub_delay * sampleTime))) * const_z;
                        //                        std::cout<<"z_delay_started gets true"<<"\n";
                        z_delay_started = true;
                    }
                else
                    z_delay =
                        z_atTrajStart + del_z * (sin(rotvel * (traj_time - pos_pub_delay * sampleTime))) * const_z;
            }

            if (traj_type == 5)  // square (TO BE IMPROVED for delay)
            {
                if (print_flag_square == 1)
                {
                    t_last = ros::Time::now().toSec();

                    ROS_INFO("--------Square selected!--------");
                    print_flag_hover_origin = 1;
                    print_flag_hover = 1;
                    print_flag_hover_lidar = 1;
                    print_flag_circle = 1;
                    print_flag_fig8 = 1;
                    print_flag_square = 0;
                    print_flag_setpoint = 1;
                    print_flag_GP = 1;
                    x_atTrajStart = pos_ref_start_msg.pose.position.x + x_hover;
                    y_atTrajStart = pos_ref_start_msg.pose.position.y + y_hover;
                    z_atTrajStart = pos_ref_start_msg.pose.position.z + z_hover;

                    x_last = x_atTrajStart;
                    y_last = y_atTrajStart;
                    z_last = z_atTrajStart;

                    x_delay_started = false;
                    y_delay_started = false;
                    z_delay_started = false;
                    x_delay_start = x;
                    y_delay_start = y;
                    z_delay_start = z;
                }
                if (std::abs(sin(rotvel * traj_time) - 1) < 0.001 || std::abs(sin(rotvel * traj_time) + 1) < 0.001)
                    x = x_atTrajStart + radius * (sin(rotvel * traj_time) < 0 ? std::floor(sin(rotvel * traj_time))
                                                                              : std::ceil(sin(rotvel * traj_time)));
                else
                    x = x_last;
                if (traj_time < 0.25 * time_period)
                {
                    y = y_atTrajStart;
                    z = z_atTrajStart;
                    t_last_z = ros::Time::now().toSec();
                }
                else
                {
                    traj_time_z = t - t_last_z;
                    if (std::abs(cos(rotvel * traj_time) - 1) < 0.001 || std::abs(cos(rotvel * traj_time) + 1) < 0.001)
                        y = y_atTrajStart + radius * (cos(rotvel * traj_time) < 0 ? std::floor(cos(rotvel * traj_time))
                                                                                  : std::ceil(cos(rotvel * traj_time)));
                    else
                        y = y_last;
                    if (std::abs(sin(rotvel * traj_time_z) - 1) < 0.001 ||
                        std::abs(sin(rotvel * traj_time_z) + 1) < 0.001)
                        z = z_atTrajStart - del_z *
                                                (sin(rotvel * traj_time_z) < 0 ? std::floor(sin(rotvel * traj_time_z))
                                                                               : std::ceil(sin(rotvel * traj_time_z))) *
                                                const_z;
                    else
                        z = z_last;
                }
                x_delay = x;
                y_delay = y;
                z_delay = z;
            }
            if (traj_type == 6)  // setpoint (TO BE IMPROVED for delay)
            {
                if (print_flag_setpoint == 1)
                {
                    t_last = ros::Time::now().toSec();
                    t_last_eachRun = ros::Time::now().toSec();

                    ROS_INFO("--------Setpoint selected!--------");
                    print_flag_hover_origin = 1;
                    print_flag_hover = 1;
                    print_flag_hover_lidar = 1;
                    print_flag_circle = 1;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;
                    print_flag_setpoint = 0;
                    print_flag_GP = 1;
                    x = x_sp_start;
                    y = y_sp_start;
                    z = z_sp_start;

                    setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);
                    R_BI.setRotation(setpoint_att_quat);
                    R_BI = R_BI.transpose();
                    R_IB = R_BI.transpose();

                    std::cout << "R_BI = ";
                    for (int i = 0; i < 3; i++)
                    {
                        for (int j = 0; j < 3; j++)
                            std::cout << R_BI[i][j] << ", ";
                        std::cout << "\n";
                    }
                    std::cout << "R_IB = ";
                    for (int i = 0; i < 3; i++)
                    {
                        for (int j = 0; j < 3; j++)
                            std::cout << R_IB[i][j] << ", ";
                        std::cout << "\n";
                    }

                    // TO BE: checked!
                    x_sp_end = x_sp_start - R_IB[0][1] * length;
                    y_sp_end = y_sp_start - R_IB[1][1] * length;
                    z_sp_end = z_sp_start - height;
                    if (z_sp_end < ALLOWED_MIN_Z)
                    {
                        ROS_WARN("Travel height is set to more than allowed!");
                        ROS_WARN("Drone will go to the lowest allowed z = %f m!", ALLOWED_MIN_Z);
                        z_sp_end = ALLOWED_MIN_Z;
                    }

                    x_B_sp_start = R_BI[0][0] * x + R_BI[0][1] * y;
                    y_B_sp_start = R_BI[1][0] * x + R_BI[1][1] * y;
                    x_B = x_B_sp_start;
                    y_B = y_B_sp_start;

                    x_B_sp_end = R_BI[0][0] * x_sp_end + R_BI[0][1] * y_sp_end;
                    y_B_sp_end = R_BI[1][0] * x_sp_end + R_BI[1][1] * y_sp_end;

                    //                    if (std::abs(x_sp_start - x_sp_end) <= 0.001)
                    //                        wall_direc = 1;
                    //                    else if (std::abs(y_sp_start - y_sp_end) <= 0.001)
                    //                        wall_direc = 2;
                    //                    else
                    //                    {
                    //                        ROS_ERROR("Start and end setpoints should be identical for either X or Y!");
                    //                        ROS_ERROR("Change to Hover and restart setpoint trajectory!");
                    //                        while (ros::ok() && traj_type == 5)
                    //                        {
                    //                            ros::spinOnce();
                    //                            rate.sleep();
                    //                        }
                    //                    }
                    // delta_z = x_wall * tan((1-overlap)*theta/2)
                    z_interval_dist =
                        wall_dist * tan(deg2rad * (1.0 - (double)req_V_overlap_percent / 100) * camera_V_FOV / 2);
                    num_turns = (int)(std::abs(z_sp_start - z_sp_end) / z_interval_dist);
                    if (num_turns % 2 != 0)
                        z_interval_dist_updated = std::abs(z_sp_start - z_sp_end) / ++num_turns;
                    else
                        z_interval_dist_updated = z_interval_dist;
                    sp_left_corner_reached_flag = true;
                    sp_right_corner_reached_flag = false;
                    sp_z_counter = 0;
                    sp_z_counter_switch = true;

                    if (!lidar_on)
                    {
                        ROS_WARN("Lidar is not started!");
                        ROS_WARN("Holding X_B = %f!", x_B_sp_start);
                        //                        if (wall_direc == 1)
                        //                            ROS_WARN("Holding X = %f!",x_sp_start);
                        //                        else if (wall_direc == 2)
                        //                            ROS_WARN("Holding Y = %f!",y_sp_start);
                    }

                    //                    std::cout<<"wall_direc = "<<wall_direc<<"\n";
                    std::cout << "num_turns = " << num_turns << "\n";
                    std::cout << "z_interval_dist = " << z_interval_dist << "\n";
                    std::cout << "z_interval_dist_updated = " << z_interval_dist_updated << "\n";

                    x_last = x;
                    y_last = y;
                    z_last = z;
                    x_delay_started = false;
                    y_delay_started = false;
                    z_delay_started = false;
                    x_delay_start = x;
                    y_delay_start = y;
                    z_delay_start = z;

                    // TO BE: removed
                    if (!use_current_pos)
                    {
                        ROS_WARN("use_current_pos is set to false!");
                        ROS_WARN("Not utilizing position feedback!");
                    }
                }

                traj_time_eachRun = ros::Time::now().toSec() - t_last_eachRun;

                if (lidar_on)
                {
                    // TO BE: removed
                    if (use_current_pos)
                        x_B = x_B_sp_start + (compute_actual_wall_dist() - wall_dist) +
                              (R_BI[0][0] * current_pos[0] + R_BI[0][1] * current_pos[1]);
                    else
                        x_B = x_B_sp_start + (compute_actual_wall_dist() - wall_dist);
                }
                else
                    x_B = x_B;
                //                    std::cout<<"x_B = "<<x_B<<"\n";
                if (z >= z_sp_end)
                {
                    if ((y_B > y_B_sp_end) && sp_left_corner_reached_flag && !sp_right_corner_reached_flag)
                    {
                        if (sp_z_counter_switch)
                        {
                            std::cout << "sp_z_counter = " << sp_z_counter++ << "\n";
                            sp_z_counter_switch = false;
                            t_last_eachRun = ros::Time::now().toSec();
                        }
                        y_B -= absvel * sampleTime;
                        z = z;
                        if (std::abs(y_B - y_B_sp_end) <= absvel * sampleTime &&
                            std::abs(traj_time_eachRun - std::abs(y_B_sp_start - y_B_sp_end) / absvel < 1))
                        {
                            std::cout << "right corner reached!\n";
                            sp_left_corner_reached_flag = false;
                            sp_right_corner_reached_flag = false;
                        }
                    }
                    else if ((y_B < y_B_sp_start) && !sp_left_corner_reached_flag && sp_right_corner_reached_flag)
                    {
                        if (sp_z_counter_switch)
                        {
                            std::cout << "sp_z_counter = " << sp_z_counter++ << "\n";
                            sp_z_counter_switch = false;
                            t_last_eachRun = ros::Time::now().toSec();
                        }
                        y_B += absvel * sampleTime;
                        z = z;
                        if (std::abs(y_B - y_B_sp_start) <= absvel * sampleTime &&
                            std::abs(traj_time_eachRun - std::abs(y_B_sp_start - y_B_sp_end) / absvel < 1))
                        {
                            std::cout << "left corner reached!\n";
                            sp_left_corner_reached_flag = false;
                            sp_right_corner_reached_flag = false;
                        }
                    }
                    else if ((!sp_left_corner_reached_flag && !sp_right_corner_reached_flag) &&
                             z > (z_sp_start - z_interval_dist_updated * sp_z_counter))
                    {
                        if (!sp_z_counter_switch)
                        {
                            sp_z_counter_switch = true;
                            t_last_eachRun = ros::Time::now().toSec();
                        }
                        y_B = y_B;
                        z -= absvel * sampleTime;
                        if (std::abs(z - (z_sp_start - z_interval_dist_updated * sp_z_counter)) <= absvel * sampleTime)
                        {
                            if (std::abs(y_B - y_B_sp_end) <= absvel * sampleTime)
                            {
                                sp_left_corner_reached_flag = false;
                                sp_right_corner_reached_flag = true;
                            }
                            else if (std::abs(y_B - y_B_sp_start) <= absvel * sampleTime)
                            {
                                sp_left_corner_reached_flag = true;
                                sp_right_corner_reached_flag = false;
                            }
                        }
                    }
                }
                else
                {
                    if (sp_z_counter_switch)
                    {
                        std::cout << "Final setpoint reached!\n";
                        std::cout << "Trajectory complete!\n";
                        sp_z_counter_switch = false;
                    }
                    y_B = y_B;
                    z = z;
                }

                //                std::cout<<"z = "<<z<<"\n";
                //                std::cout<<"(z_sp_start-z_interval_dist_updated*sp_z_counter) = "<<(z_sp_start-z_interval_dist_updated*sp_z_counter)<<"\n";
                //                std::cout<<"std::abs(z-(z_sp_start-z_interval_dist_updated*sp_z_counter)) = "<<std::abs(z-(z_sp_start-z_interval_dist_updated*sp_z_counter))<<"\n";
                //                std::cout<<"absvel*sampleTime = "<<absvel*sampleTime<<"\n";
                ////                std::cout<<"std::abs(z-(z_sp_start-z_interval_dist_updated*sp_z_counter))<=absvel*sampleTime = "<<std::abs(z-(z_sp_start-z_interval_dist_updated*sp_z_counter))<=absvel*sampleTime? 1:0<<"\n";

                //                std::cout<<"sp_left_corner_reached_flag = "<<sp_left_corner_reached_flag<<"\n";
                //                std::cout<<"sp_right_corner_reached_flag = "<<sp_right_corner_reached_flag<<"\n";
                //                std::cout<<"sp_z_counter = "<<sp_z_counter<<"\n";
                //                std::cout<<"sp_z_counter_switch = "<<sp_z_counter_switch<<"\n";

                x = R_IB[0][0] * x_B + R_IB[0][1] * y_B;
                y = R_IB[1][0] * x_B + R_IB[1][1] * y_B;

                x_delay = x;
                y_delay = y;
                z_delay = z;
            }

            //add G.P trajectory hakims part
            if (traj_type == 7)
            {
                if (print_flag_GP == 1)
                {
                    ROS_INFO("--------Global Planner selected!--------");
                    print_flag_hover_origin = 1;
                    print_flag_hover = 1;
                    print_flag_hover_lidar = 1;
                    print_flag_circle = 1;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;
                    print_flag_setpoint = 1;
                    print_flag_GP = 0;
                }
                x = desired_pos.pose.position.x;
                y = desired_pos.pose.position.y;
                z = desired_pos.pose.position.z;

                
                //x=0.0;
                //y=0.0;
                //z=2.0;

                //if x-x_d

                //x_delay = x;
                //y_delay = y;
                // z_delay = z;
            }

            ref_yaw_msg.data = compute_ref_yaw();
            setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);

            if (!lidar_on)
            {
                print_Llidar_nan_flag = 0;
                print_Clidar_nan_flag = 0;
                print_Rlidar_nan_flag = 0;
                print_ALLlidar_nan_flag = 0;
            }
        }
        else if (traj_on == 1 || land_flag)
        {
            if (print_flag_traj_on == 0)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Holding position at [x,y,z]: %.2f,%.2f,%.2f", x, y, z);
                print_flag_traj_on = 1;
            }
            x = x;
            y = y;
            z = z;

            x_delay = x;
            y_delay = y;
            z_delay = z;

            if (land_flag)
                ref_yaw_msg.data = ref_yaw_msg.data;
            else
                ref_yaw_msg.data = compute_ref_yaw();
            setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);

            t_last = ros::Time::now().toSec();

            if (!lidar_on)
            {
                print_Llidar_nan_flag = 0;
                print_Clidar_nan_flag = 0;
                print_Rlidar_nan_flag = 0;
                print_ALLlidar_nan_flag = 0;
            }
        }
        else
        {
            if (traj_started_flag)
            {
                //                pos_ref_start_msg.pose.position.z = 0.0;
                traj_started_flag = false;
            }

            if (print_flag_traj_on == 0)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Default reference position only!");
                ROS_INFO("Position reference x =  %.2f m!", pos_ref_start_msg.pose.position.x);
                ROS_INFO("Position reference y =  %.2f m!", pos_ref_start_msg.pose.position.y);
                ROS_INFO("Position reference z =  %.2f m!", pos_ref_start_msg.pose.position.z);
                print_flag_traj_on = 1;
            }

            if (!max_z_on)
            {
                x = pos_ref_start_msg.pose.position.x;
                y = pos_ref_start_msg.pose.position.y;
            }
            z = pos_ref_start_msg.pose.position.z;

            x_delay = x;
            y_delay = y;
            z_delay = z;

            ref_yaw_msg.data = ref_yaw_msg.data;
            setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);

            t_last = ros::Time::now().toSec();
            //            std::cout<<"t_last = "<<t_last<<"\n";

            if (!max_z_on)
                climbed_flag = false;
            landed_flag = false;
            if (!lidar_on)
            {
                print_Llidar_nan_flag = 0;
                print_Clidar_nan_flag = 0;
                print_Rlidar_nan_flag = 0;
                print_ALLlidar_nan_flag = 0;
            }
            print_flag_hover_origin = 1;
            print_flag_hover = 1;
            print_flag_hover_lidar = 1;
            print_flag_circle = 1;
            print_flag_fig8 = 1;
            print_flag_square = 1;
            print_flag_setpoint = 1;
            print_flag_GP = 1;
        }

        if (land_flag)
        {
            while (ros::ok() && land_flag && !landed_flag && z_delay > 0.0)
            {
                if (print_flag_land == 0)
                {
                    ROS_INFO("---------------------------------");
                    ROS_INFO("Landing initialized!");
                    print_flag_land = 1;

                    z_delay_start = z;
                }
                x = x;
                y = y;
                z = z > 0 ? z - land_rate * sampleTime : 0;

                x_delay = x;
                y_delay = y;
                z_delay = std::abs(z - z_delay_start) < land_rate * pos_pub_delay * sampleTime
                              ? z_delay_start
                              : z_delay - land_rate * sampleTime;

                //u = (x - x_last) / sampleTime;
                //v = (y - y_last) / sampleTime;
                //w = (z - z_last) / sampleTime;
                
                //publish_inspection_point();
               
                ref_yaw_msg.data = compute_ref_yaw();
                if (!pub_setpoint_pos)
                {
                    reftrajectory_msg.x = x;
                    reftrajectory_msg.y = y;
                    reftrajectory_msg.z = z;
                    ref_pos_pub.publish(reftrajectory_msg);
                    reftrajectory_delay_msg.x = x_delay;
                    reftrajectory_delay_msg.y = y_delay;
                    reftrajectory_delay_msg.z = z_delay;
                    ref_pos_delay_pub.publish(reftrajectory_delay_msg);

                    //reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
                    //reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
                    //reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);




                    //ref_vel_pub.publish(reftrajectory_vel_msg);

                    //ref_yaw_pub.publish(ref_yaw_msg);
                    // For rviz visualization:
                    setpoint_pos_msg.header.stamp = ros::Time::now();
                    setpoint_pos_msg.header.frame_id = "map";
                    setpoint_pos_msg.pose.position.x = x;
                    setpoint_pos_msg.pose.position.y = y;
                    setpoint_pos_msg.pose.position.z = z;

                    setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);
                    setpoint_pos_msg.pose.orientation.x = desired_pos.pose.orientation.x;
                    setpoint_pos_msg.pose.orientation.y = desired_pos.pose.orientation.x;
                    setpoint_pos_msg.pose.orientation.z = desired_pos.pose.orientation.x;
                    setpoint_pos_msg.pose.orientation.w = desired_pos.pose.orientation.x;;
                    ref_pose_pub.publish(setpoint_pos_msg);
                }
                else
                {
                    setpoint_pos_msg.header.stamp = ros::Time::now();
                    setpoint_pos_msg.header.frame_id = "map";
                    setpoint_pos_msg.pose.position.x = x;
                    setpoint_pos_msg.pose.position.y = y;
                    setpoint_pos_msg.pose.position.z = z;

                    setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);
                    //setpoint_pos_msg.pose.orientation.x = setpoint_att_quat.getX();
                    //setpoint_pos_msg.pose.orientation.y = setpoint_att_quat.getY();
                    //setpoint_pos_msg.pose.orientation.z = setpoint_att_quat.getZ();
                    //setpoint_pos_msg.pose.orientation.w = setpoint_att_quat.getW();


 

                    setpoint_pos_msg.pose.orientation.x = desired_pos.pose.orientation.x;
;
                    setpoint_pos_msg.pose.orientation.y = desired_pos.pose.orientation.y;
;
                    setpoint_pos_msg.pose.orientation.z = desired_pos.pose.orientation.z;
;
                    setpoint_pos_msg.pose.orientation.w =desired_pos.pose.orientation.w;
;


                    setpoint_pos_pub.publish(setpoint_pos_msg);
                    ref_pose_pub.publish(setpoint_pos_msg);
                }

                if (z_delay <= 0.0)
                {
                    if (print_flag_land == 1)
                    {
                        ROS_INFO("Landing complete!");
                        print_flag_land = 0;
                    }
                    //                        pos_ref_start_msg.pose.position.z = 0.0;
                    climbed_flag = false;
                    landed_flag = true;
                    t_last = ros::Time::now().toSec();
                }

                x_last = x;
                y_last = y;
                z_last = z;

                traj_on_pub.publish(traj_on_msg);
                reg_on_pub.publish(reg_on_msg);
                ros::spinOnce();
                rate.sleep();
            }
        }
        publish_inspection_point();
        //double u_global = (x - x_last) / sampleTime;
        //double v_global = (y - y_last) / sampleTime;
       // double w_global = (z - z_last) / sampleTime;
        double u_global = vel_d_g.pose.position.x;
        double v_global =  vel_d_g.pose.position.y;
        double w_global =  vel_d_g.pose.position.z;
        
       


        // desired velocity data in body frame
        tf::Matrix3x3 rotational_matrix_BI(current_att_quat);
        rotational_matrix_BI = rotational_matrix_BI.transpose();

        u = rotational_matrix_BI[0][0] * u_global + rotational_matrix_BI[0][1] * v_global +
            rotational_matrix_BI[0][2] * w_global;
        v = rotational_matrix_BI[1][0] * u_global + rotational_matrix_BI[1][1] * v_global +
            rotational_matrix_BI[1][2] * w_global;
        w = rotational_matrix_BI[2][0] * u_global + rotational_matrix_BI[2][1] * v_global +
            rotational_matrix_BI[2][2] * w_global;





        //        std::cout<<"absolute vel along x & y = "<<sqrt(u*u + v*v)<<"\n";

        x_last = x;
        y_last = y;
        z_last = z;

        // TO BE: removed!
        //        lidar_read_filtered_pub.publish(Clidar_read_filtered_msg);
        std_msgs::Float64 v_d_m;
        v_d_m.data = v_d;
        //drone_velocity_pub.publish(v_d_m);






        reftrajectory_vel_msg.x = u*v_d;
        reftrajectory_vel_msg.y = v*v_d;
        reftrajectory_vel_msg.z = w*v_d;
        ref_vel_pub.publish(reftrajectory_vel_msg);

        publish_inspection_point();



        if (!pub_setpoint_pos)
        {
            reftrajectory_msg.x = x;
            reftrajectory_msg.y = y;
            reftrajectory_msg.z = z;
            ref_pos_pub.publish(reftrajectory_msg);
            reftrajectory_delay_msg.x = x_delay;
            reftrajectory_delay_msg.y = y_delay;
            reftrajectory_delay_msg.z = z_delay;
            ref_pos_delay_pub.publish(reftrajectory_delay_msg);

            reftrajectory_vel_msg.x = u*v_d;
            reftrajectory_vel_msg.y = v*v_d;
            reftrajectory_vel_msg.z = w*v_d;
            //ref_vel_pub.publish(reftrajectory_vel_msg);

            ref_yaw_pub.publish(ref_yaw_msg);
            // For rviz visualization:
            setpoint_pos_msg.header.stamp = ros::Time::now();
            setpoint_pos_msg.header.frame_id = "map";
            setpoint_pos_msg.pose.position.x = x;
            setpoint_pos_msg.pose.position.y = y;
            setpoint_pos_msg.pose.position.z = z;

            setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);
            setpoint_pos_msg.pose.orientation.x = desired_pos.pose.orientation.x;
            setpoint_pos_msg.pose.orientation.y = desired_pos.pose.orientation.y;
            setpoint_pos_msg.pose.orientation.z = desired_pos.pose.orientation.z;
            setpoint_pos_msg.pose.orientation.w = desired_pos.pose.orientation.w;
            ref_pose_pub.publish(setpoint_pos_msg);
        }
        else
        {
            setpoint_pos_msg.header.stamp = ros::Time::now();
            setpoint_pos_msg.header.frame_id = "map";
            setpoint_pos_msg.pose.position.x = x;
            setpoint_pos_msg.pose.position.y = y;
            setpoint_pos_msg.pose.position.z = z;

            setpoint_att_quat.setRPY(0, 0, ref_yaw_msg.data);
            setpoint_pos_msg.pose.orientation.x = desired_pos.pose.orientation.x;
            setpoint_pos_msg.pose.orientation.y = desired_pos.pose.orientation.y;
            setpoint_pos_msg.pose.orientation.z = desired_pos.pose.orientation.z;

            setpoint_pos_msg.pose.orientation.w = desired_pos.pose.orientation.w;

            setpoint_pos_pub.publish(setpoint_pos_msg);
            ref_pose_pub.publish(setpoint_pos_msg);
        }

        traj_on_pub.publish(traj_on_msg);
        reg_on_pub.publish(reg_on_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void publish_inspection_point()
{
    geometry_msgs::PoseStamped ref_point;
    ref_point.header.stamp = ros::Time::now();
    ref_point.header.frame_id = "map";
    geometry_msgs::PoseStamped ref_normal;
    ref_normal.header.stamp = ros::Time::now();
    ref_normal.header.frame_id = "map";
    if (point_tracking_on)
    {

                    
                    float px_p = point.pose.position.x;
                    float py_p = point.pose.position.y;
                    float pz_p = point.pose.position.z;
                    float nx = normal.pose.position.x;
                    float ny = normal.pose.position.y;
                    float nz = normal.pose.position.z;
                    
                    float px_p_delayed = point_delayed.pose.position.x;
                    float py_p_delayed = point_delayed.pose.position.y;
                    float pz_p_delayed = point_delayed.pose.position.z;

                    ref_normal.pose.position.x = nx;
                    ref_normal.pose.position.y = ny;
                    ref_normal.pose.position.z = 0;

                    ref_point.pose.position.x = px_p;
                    ref_point.pose.position.y = py_p;
                    ref_point.pose.position.z = pz_p;

                   //test 

                    //ref_point.pose.position.x=px_test.data;
                    //ref_point.pose.position.y=py_test.data;
                    //ref_point.pose.position.z=z;
                    //ref_normal.pose.position.x = nx_test.data;
                    //ref_normal.pose.position.y = ny_test.data;
                    //ref_normal.pose.position.z = 0;

                   // ref_point.pose.position.x = px;
                   // ref_point.pose.position.y = py;
                   // ref_point.pose.position.z = pz;
                    
    }
    else
    {
        //ref_point.pose.position.x = -10.0;
        //ref_point.pose.position.y = 1.1;
        ref_point.pose.position.x = px;
        ref_point.pose.position.y = py;


        ref_point.pose.position.z = z;
        ref_normal.pose.position.x = 1.0;
        ref_normal.pose.position.y = 0.0;
        ref_normal.pose.position.z = 0.0;
    }

    //ROS_INFO("point x1 %f", point.pose.position.x);
    //ROS_INFO("point x2 %f", px_p);
    //ROS_INFO("point x3 %f", ref_point.pose.position.x);
    point_to_view_pub.publish(ref_point);
    norm_desired_pub.publish(ref_normal);
}

double compute_ref_yaw()
{
    if (!adaptive_yaw_on)
    {
        ref_yaw_adaptive = deg2rad * yaw_hover;
    }
    else if (std::isnan(Llidar_read_data) || std::isnan(Rlidar_read_data))
    {
        if (print_Llidar_nan_flag == 0 && print_Rlidar_nan_flag == 0)
        {
            ROS_WARN("Either Llidar or Rlidar is showing nan values!");
            if (!use_current_pos)
                ROS_WARN("Maintaining yaw = %f deg!", rad2deg * ref_yaw_adaptive);
            else
                ROS_WARN("Maintaining yaw = %f deg!", rad2deg * current_att[2]);
            if (std::isnan(Llidar_read_data))
                print_Llidar_nan_flag = 1;
            if (std::isnan(Rlidar_read_data))
                print_Rlidar_nan_flag = 1;
        }
        if (!use_current_pos)
            ref_yaw_adaptive = ref_yaw_adaptive;
        else
            ref_yaw_adaptive = current_att[2];
    }
    else
    {
        if (print_Llidar_nan_flag == 1 || print_Rlidar_nan_flag == 1)
        {
            ROS_WARN("Normal function resumed for L and R lidars!");
            ROS_WARN("Maintaining adaptive yaw!");
            print_Llidar_nan_flag = 0;
            print_Rlidar_nan_flag = 0;
        }
        if (std::abs(Llidar_read_data - Rlidar_read_data) > 0.1)
        {
            if (Llidar_read_data < Rlidar_read_data)
                ref_yaw_adaptive += MAX_ALLOWED_YAWRATE * sampleTime;
            else
                ref_yaw_adaptive -= MAX_ALLOWED_YAWRATE * sampleTime;
        }
    }
    return ref_yaw_adaptive;
}

double compute_actual_wall_dist()
{
    if (!std::isnan(Clidar_read_data))
    {
        if (print_Clidar_nan_flag == 1)
        {
            ROS_WARN("Normal function resumed for Clidar!");
            t_Clidar_return_start = ros::Time::now().toSec();
        }
        print_Clidar_nan_flag = 0;
        print_ALLlidar_nan_flag = 0;

        if (use_sonar)
        {
            if (ros::Time::now().toSec() - t_Clidar_return_start > 1.0)
                return Clidar_read_data;
            else
                return sonar_read_data - (sonar_read_data - Clidar_read_data) *
                                             sin((M_PI / 2) * (ros::Time::now().toSec() - t_Clidar_return_start));
        }
        else
            return Clidar_read_data;
    }
    else if (use_sonar && !std::isnan(sonar_read_data))
    {
        if (print_Clidar_nan_flag == 0)
        {
            ROS_WARN("Clidar is showing nan values!");
            ROS_WARN("Taking the sonar data as actual distance!");
            print_Clidar_nan_flag = 1;
            t_Clidar_lost_start = ros::Time::now().toSec();
        }
        if (print_sonar_nan_flag == 1)
            ROS_WARN("Normal function resumed for sonar!");
        print_Clidar_nan_flag = 1;
        print_sonar_nan_flag = 0;
        print_ALLlidar_nan_flag = 0;

        if (ros::Time::now().toSec() - t_Clidar_lost_start > 1.0)
            return sonar_read_data;
        else
            return Clidar_read_data_unfiltered[Clidar_read_data_unfiltered.size() - 2] -
                   (Clidar_read_data_unfiltered[Clidar_read_data_unfiltered.size() - 2] - sonar_read_data) *
                       sin((M_PI / 2) * (ros::Time::now().toSec() - t_Clidar_lost_start));
    }
    else
    {
        if (print_Clidar_nan_flag == 0 || print_sonar_nan_flag == 0)
            ROS_WARN("Clidar and sonar are showing nan values!");

        if (std::isnan(Llidar_read_data) && std::isnan(Rlidar_read_data))
        {
            if (print_ALLlidar_nan_flag == 0)
            {
                ROS_WARN("All lidars and sonar are showing nan values!");
                ROS_WARN("Taking actual distance = %f for 10 secs before landing!", wall_dist);
                print_ALLlidar_nan_flag = 1;
                print_Llidar_nan_flag = 1;
                print_Clidar_nan_flag = 1;
                print_sonar_nan_flag = 1;
                print_Rlidar_nan_flag = 1;
                t_ALLlidar_lost_start = ros::Time::now().toSec();
            }
            if (ros::Time::now().toSec() - t_ALLlidar_lost_start < 10)
                return wall_dist;
            else
            {
                ROS_WARN("Neither any lidar nor sonar showed any reading for the past 10 secs!");
                ROS_WARN("Landing is requested!");
                land_flag = true;
                return wall_dist;
            }
        }
        else if (std::isnan(Llidar_read_data) || std::isnan(Rlidar_read_data))
        {
            if (std::isnan(Llidar_read_data))
            {
                if (print_Llidar_nan_flag == 0)
                {
                    ROS_WARN("Llidar is also showing nan values!");
                    ROS_WARN("Rlidar is taken as actual distance!");
                    print_Llidar_nan_flag = 1;
                    print_Clidar_nan_flag = 1;
                    print_sonar_nan_flag = 1;
                }
                if (print_Rlidar_nan_flag == 1 || print_ALLlidar_nan_flag == 1)
                {
                    ROS_WARN("Normal function resumed for Rlidar!");
                    print_Rlidar_nan_flag = 0;
                    print_ALLlidar_nan_flag = 0;
                    print_Clidar_nan_flag = 1;
                    print_sonar_nan_flag = 1;
                }

                return Rlidar_read_data * cos(deg2rad * DEG_BET_EACH_LIDAR);
            }
            else if (std::isnan(Rlidar_read_data))
            {
                if (print_Rlidar_nan_flag == 0)
                {
                    ROS_WARN("Rlidar is also showing nan values!");
                    ROS_WARN("Llidar is taken as actual distance!");
                    print_Rlidar_nan_flag = 1;
                    print_Clidar_nan_flag = 1;
                    print_sonar_nan_flag = 1;
                }
                if (print_Llidar_nan_flag == 1 || print_ALLlidar_nan_flag == 1)
                {
                    ROS_WARN("Normal function resumed for Llidar!");
                    print_Llidar_nan_flag = 0;
                    print_ALLlidar_nan_flag = 0;
                    print_Clidar_nan_flag = 1;
                    print_sonar_nan_flag = 1;
                }

                return Llidar_read_data * cos(deg2rad * DEG_BET_EACH_LIDAR);
            }
        }
        else
        {
            if (print_Clidar_nan_flag == 0 || print_sonar_nan_flag == 0)
            {
                ROS_WARN("Taking the average of L and R lidars as actual distance!");
                print_Clidar_nan_flag = 1;
                print_sonar_nan_flag = 1;
            }
            if (print_Llidar_nan_flag == 1 || print_Rlidar_nan_flag == 1)
            {
                ROS_WARN("Normal function resumed for L and R lidars!");
                print_Llidar_nan_flag = 0;
                print_Rlidar_nan_flag = 0;
                print_ALLlidar_nan_flag = 0;
            }

            return (Llidar_read_data * cos(deg2rad * DEG_BET_EACH_LIDAR) +
                    Rlidar_read_data * cos(deg2rad * DEG_BET_EACH_LIDAR)) /
                   2;
        }
    }
}
