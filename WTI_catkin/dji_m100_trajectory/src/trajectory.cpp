/**
 * @file   trajectory.cpp
 * @author Mohit Mehndiratta
 * @date   April 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <trajectory.h>
#include<sensor_msgs/Range.h>
#include <dji_m100_trajectory/set_trajectoryConfig.h>

double sampleTime = 0.01;

void dynamicReconfigureCallback(dji_m100_trajectory::set_trajectoryConfig &config, uint32_t level)
{
    traj_start = config.traj_start;
    max_z_start = config.max_z_start;
    lidar_start = config.lidar_start;
    climb_flag = config.climb;
    land_flag = config.land;
    change_z = config.change_z;
    pub_setpoint_pos = config.pub_on_setpoint_position;

    traj_type = config.traj_type;
    pos_pub_delay = config.pos_pub_delay;
    max_z = config.max_z;
    x_hover = config.x_hover;
    y_hover = config.y_hover;
    z_hover = config.z_hover;
    yaw_hover = config.yaw_hover;
    wall_dist = config.wall_dist;
    x_sp_start = config.x_sp_start;
    y_sp_start = config.y_sp_start;
    z_sp_start = config.z_sp_start;
    x_sp_end = config.x_sp_end;
    y_sp_end = config.y_sp_end;
    z_sp_end = config.z_sp_end;
    z_interval_dist = config.z_interval_dist;
    del_z = config.del_z;
    radius = config.des_radius;
    absvel = config.des_velocity;

    rotvel = absvel/radius;
    time_period = 2*M_PI/rotvel;

    climb_rate = config.climb_rate;
    land_rate = config.land_rate;
}

// Callback function
std::vector<double> current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
}
std::vector<double> lidar_read_data_unfiltered(10,0.0);
double lidar_read_data;
std_msgs::Float64 lidar_read_filtered_msg;
void lidar_read_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    int i = 0;
    while (i < lidar_read_data_unfiltered.size()-1)
    {
        lidar_read_data_unfiltered[i] = lidar_read_data_unfiltered[i+1];
        i++;
    }
    if (std::isnan(msg->range) || std::isinf(msg->range))
        lidar_read_data_unfiltered[i] = lidar_read_data_unfiltered[i-1];
    else if (msg->range<msg->min_range)
        lidar_read_data_unfiltered[i] = msg->min_range;
//    else if (msg->range>msg->max_range)
//        lidar_read_data = msg->max_range;
    else if (msg->range>10)
        lidar_read_data_unfiltered[i] = 10;
    else
        lidar_read_data_unfiltered[i] = msg->range;

    lidar_read_data = 0.0;
//    for (int i=0; i<lidar_read_data_unfiltered.size()-1; ++i)
//        lidar_read_data += lidar_read_data_unfiltered[i];
//    lidar_read_data += lidar_read_data_unfiltered.size()*lidar_read_data_unfiltered[i];
//    lidar_read_data = lidar_read_data/(lidar_read_data_unfiltered.size() +
//                                       lidar_read_data_unfiltered.size()-1);
    for (int i=0; i<lidar_read_data_unfiltered.size(); ++i)
        lidar_read_data += lidar_read_data_unfiltered[i];
    lidar_read_data = lidar_read_data/(lidar_read_data_unfiltered.size());


    lidar_read_filtered_msg.data = lidar_read_data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "m100_trajectory");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<dji_m100_trajectory::set_trajectoryConfig> server;
    dynamic_reconfigure::Server<dji_m100_trajectory::set_trajectoryConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    // Roslaunch parameters
    ros::param::get("lidar_topic",lidar_topic);
    ros::param::get("use_current_pos",use_current_pos);

    // Publisher
    ref_pos_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/position", 1);
    ref_pos_delay_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/position_delayed", 1);
    ref_vel_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/velocity", 1);
    ref_yaw_pub = nh.advertise<std_msgs::Float64>("ref_trajectory/yaw", 1);
    setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    traj_on_pub = nh.advertise<std_msgs::Bool>("trajectory_on", 1);
    ros::Publisher lidar_read_filtered_pub = nh.advertise<std_msgs::Float64>("range_filter", 1);

    // Subscriber
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/mocap/pose", 1, pos_cb);
    lidar_read_sub = nh.subscribe<sensor_msgs::Range>(lidar_topic, 1, lidar_read_cb);

    ros::Rate rate(1/sampleTime);

    pos_ref_start_msg.pose.position.x = 0;
    pos_ref_start_msg.pose.position.y = 0;
    if (max_z_start)
        pos_ref_start_msg.pose.position.z = max_z;
    else
        pos_ref_start_msg.pose.position.z = 0;

    traj_start_msg.data = 0.0;

    while(ros::ok())
    {
        traj_start_msg.data = traj_start;

        if (max_z_start &&  pos_ref_start_msg.pose.position.z != max_z
                        && !landed_flag && traj_start != 1)
        {
            pos_ref_start_msg.pose.position.z = max_z;
            print_flag_traj_start = 0;

        }
        else if(!max_z_start && pos_ref_start_msg.pose.position.z != 0.0
                             && traj_start != 1)
        {
            pos_ref_start_msg.pose.position.z = 0.0;
            print_flag_traj_start = 0;
        }

        if(change_z == 1)
        {
            if (print_flag_changez == 1)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Changing z by %.2f m!",del_z);
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

        if(traj_start == 1 && !landed_flag)
        {
//            traj_start_msg.data = traj_start;
            if(!traj_started_flag)
                traj_started_flag = true;

            t = ros::Time::now().toSec();
            traj_time = t - t_last;

            if(climb_flag)
            {
                while(ros::ok() && !climbed_flag && z_delay < max_z)
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
                    z = z < max_z ? z + climb_rate*sampleTime : max_z;

                    x_delay = x;
                    y_delay = y;
                    z_delay = std::abs(z - z_delay_start) < climb_rate*pos_pub_delay*sampleTime ? z_delay_start : z_delay + climb_rate*sampleTime;

                    u = (x - x_last)/sampleTime;
                    v = (y - y_last)/sampleTime;
                    w = (z - z_last)/sampleTime;

                    ref_yaw.data = deg2rad*yaw_hover;
                    if(!pub_setpoint_pos)
                    {
                        reftrajectory_msg.x = x;
                        reftrajectory_msg.y = y;
                        reftrajectory_msg.z = z;
                        ref_pos_pub.publish(reftrajectory_msg);
                        reftrajectory_delay_msg.x = x_delay;
                        reftrajectory_delay_msg.y = y_delay;
                        reftrajectory_delay_msg.z = z_delay;
                        ref_pos_delay_pub.publish(reftrajectory_delay_msg);

                        reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
                        reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
                        reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);
                        ref_vel_pub.publish(reftrajectory_vel_msg);

                        ref_yaw_pub.publish(ref_yaw);
                    }
                    else
                    {
                        setpoint_pos_msg.header.stamp = ros::Time::now();
                        setpoint_pos_msg.pose.position.x = x;
                        setpoint_pos_msg.pose.position.y = y;
                        setpoint_pos_msg.pose.position.z = z;

                        setpoint_att_quat.setRPY(0,0,ref_yaw.data);
                        setpoint_pos_msg.pose.orientation.x = setpoint_att_quat.getX();
                        setpoint_pos_msg.pose.orientation.y = setpoint_att_quat.getY();
                        setpoint_pos_msg.pose.orientation.z = setpoint_att_quat.getZ();
                        setpoint_pos_msg.pose.orientation.w = setpoint_att_quat.getW();
                        setpoint_pos_pub.publish(setpoint_pos_msg);
                    }

                    if(z_delay >= max_z)
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

                    traj_on_pub.publish(traj_start_msg);
                    ros::spinOnce();
                    rate.sleep();
                }
            }

            if(climbed_flag && !landed_flag)
            {
                pos_ref_start_msg.pose.position.z = max_z;
            }

            if (print_flag_traj_start == 1)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Reference trajectory started!");
                print_flag_traj_start = 0;
            }

            if(traj_type == 0) // hover at origin
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

            if(traj_type == 1) // hover
            {
                if (print_flag_hover == 1)
                {
                    t_last = ros::Time::now().toSec();

                    ROS_INFO("--------Hover selected!--------");
                    print_flag_hover_origin = 1;
                    print_flag_hover = 0;
                    print_flag_hover_lidar = 1;
                    print_flag_circle = 1;
                    print_flag_fig8 = 1;
                    print_flag_square = 1;
                    print_flag_setpoint = 1;

                    x_atTrajStart = x;
                    y_atTrajStart = y;
                    z_atTrajStart = z;

                    x_B_atTrajStart = R_BI[0][0]*x_atTrajStart + R_BI[0][1]*y_atTrajStart;
                    y_B_atTrajStart = R_BI[1][0]*x_atTrajStart + R_BI[1][1]*y_atTrajStart;
                    x_B = x_B_atTrajStart;
                    y_B = y_B_atTrajStart;

                    z_delay_started = false;
                    z_delay_start = z;
                }
                x = x_atTrajStart + x_hover;
                y = y_atTrajStart + y_hover;
                z = z_atTrajStart + z_hover - del_z*(sin(rotvel*traj_time)) * const_z;
                x_delay = x;
                y_delay = y;
                if (!z_delay_started)
                    if (std::abs(z - z_delay_start) < del_z*(sin(rotvel*(pos_pub_delay*sampleTime))) * const_z)
                        z_delay = z_delay_start;
                    else
                    {
                        z_delay = z_atTrajStart + z_hover - del_z*(sin(rotvel*(traj_time - pos_pub_delay*sampleTime))) * const_z;
//                        std::cout<<"z_delay_started gets true"<<"\n";
                        z_delay_started = true;
                    }
                else
                    z_delay = z_atTrajStart + z_hover - del_z*(sin(rotvel*(traj_time - pos_pub_delay*sampleTime))) * const_z;
            }

            if(traj_type == 2) // hover_lidar
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

                    x_atTrajStart = x;
                    y_atTrajStart = y;
                    z_atTrajStart = z;

                    R_BI.setRotation(setpoint_att_quat);
                    R_BI = R_BI.transpose();
                    R_IB = R_BI.transpose();

                    std::cout<<"R_BI = ";
                    for (int i=0; i<3; i++)
                    {
                        for (int j=0; j<3; j++)
                            std::cout<<R_BI[i][j]<<", ";
                        std::cout<<"\n";
                    }
                    std::cout<<"R_IB = ";
                    for (int i=0; i<3; i++)
                    {
                        for (int j=0; j<3; j++)
                            std::cout<<R_IB[i][j]<<", ";
                        std::cout<<"\n";
                    }

                    x_B_atTrajStart = R_BI[0][0]*x_atTrajStart + R_BI[0][1]*y_atTrajStart;
                    y_B_atTrajStart = R_BI[1][0]*x_atTrajStart + R_BI[1][1]*y_atTrajStart;
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
                if (lidar_start)
                {
//                    x_B = x_B_atTrajStart + (lidar_read_data-wall_dist);
//                    y_B = y_B_atTrajStart + y_hover + (y_B_atTrajStart -
//                                                       (R_BI[1][0]*current_pos[0] +
//                                                        R_BI[1][1]*current_pos[1]));
//                    x = R_IB[0][0]*x_B + R_IB[0][1]*y_B + current_pos[0];
//                    y = R_IB[1][0]*x_B + R_IB[1][1]*y_B + current_pos[1];
//                    x_B = x_B_atTrajStart + (lidar_read_data-wall_dist) +
//                                            (R_BI[0][0]*current_pos[0] +
//                                             R_BI[0][1]*current_pos[1]);
                    // TO BE: removed
                    if (use_current_pos)
                        x_B = x_B_atTrajStart + (lidar_read_data-wall_dist) +
                                                (R_BI[0][0]*current_pos[0] +
                                                 R_BI[0][1]*current_pos[1]);
                    else
                        x_B = x_B_atTrajStart + (lidar_read_data-wall_dist);
                    y_B = y_B_atTrajStart + y_hover;
                    x = R_IB[0][0]*x_B + R_IB[0][1]*y_B;
                    y = R_IB[1][0]*x_B + R_IB[1][1]*y_B;
                }
                else
                {
                    x = x_atTrajStart + x_hover;
                    y = y_atTrajStart + y_hover;
                }
                z = z_atTrajStart + z_hover - del_z*(sin(rotvel*traj_time)) * const_z;
                x_delay = x;
                y_delay = y;
                if (!z_delay_started)
                    if (std::abs(z - z_delay_start) < del_z*(sin(rotvel*(pos_pub_delay*sampleTime))) * const_z)
                        z_delay = z_delay_start;
                    else
                    {
                        z_delay = z_atTrajStart + z_hover - del_z*(sin(rotvel*(traj_time - pos_pub_delay*sampleTime))) * const_z;
//                        std::cout<<"z_delay_started gets true"<<"\n";
                        z_delay_started = true;
                    }
                else
                    z_delay = z_atTrajStart + z_hover - del_z*(sin(rotvel*(traj_time - pos_pub_delay*sampleTime))) * const_z;
            }

            if(traj_type == 3) // circle
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
                x = x_atTrajStart + radius*sin(rotvel*traj_time);
                if (!x_delay_started)
                    if (std::abs(x - x_delay_start) < radius*(sin(rotvel*(pos_pub_delay*sampleTime))))
                        x_delay = x_delay_start;
                    else
                    {
                        x_delay = x_atTrajStart + radius*sin(rotvel*(traj_time - pos_pub_delay*sampleTime));
//                        std::cout<<"x_delay_started gets true"<<"\n";
                        x_delay_started = true;
                    }
                else
                    x_delay = x_atTrajStart + radius*sin(rotvel*(traj_time - pos_pub_delay*sampleTime));

                if(traj_time < 0.25*time_period)
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
                    y = y_atTrajStart + radius*cos(rotvel*traj_time);
                    z = z_atTrajStart - del_z*((sin(rotvel*traj_time_z))) * const_z;

                    if (!y_delay_started)
                        if (std::abs(y - y_delay_start) < radius*(sin(rotvel*(pos_pub_delay*sampleTime))))
                            y_delay = y_delay_start;
                        else
                        {
                            y_delay = y_atTrajStart + radius*cos(rotvel*(traj_time - pos_pub_delay*sampleTime));
//                            std::cout<<"y_delay_started gets true"<<"\n";
                            y_delay_started = true;
                        }
                    else
                        y_delay = y_atTrajStart + radius*cos(rotvel*(traj_time - pos_pub_delay*sampleTime));

                    if (!z_delay_started)
                        if (std::abs(z - z_delay_start) < del_z*(sin(rotvel*(pos_pub_delay*sampleTime))) * const_z)
                            z_delay = z_delay_start;
                        else
                        {
                            z_delay = z_atTrajStart - del_z*(sin(rotvel*(traj_time_z - pos_pub_delay*sampleTime))) * const_z;
//                            std::cout<<"z_delay_started gets true"<<"\n";
                            z_delay_started = true;
                        }
                    else
                        z_delay = z_atTrajStart - del_z*(sin(rotvel*(traj_time_z - pos_pub_delay*sampleTime))) * const_z;
                }
            }

            if(traj_type == 4) // figure 8
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
                x = x_atTrajStart - radius*cos(rotvel*traj_time + M_PI/2);
                if (!x_delay_started)
                    if (std::abs(x - x_delay_start) < radius*(sin(rotvel*(pos_pub_delay*sampleTime))))
                        x_delay = x_delay_start;
                    else
                    {
                        x_delay = x_atTrajStart - radius*cos(rotvel*(traj_time - pos_pub_delay*sampleTime) + M_PI/2);
//                        std::cout<<"x_delay_started gets true"<<"\n";
                        x_delay_started = true;
                    }
                else
                    x_delay = x_atTrajStart - radius*cos(rotvel*(traj_time - pos_pub_delay*sampleTime) + M_PI/2);

                y = y_atTrajStart + (radius/2)*sin(2*rotvel*traj_time);
                if (!y_delay_started)
                    if (std::abs(y - y_delay_start) < (radius/2)*(sin(2*rotvel*(pos_pub_delay*sampleTime))))
                        y_delay = y_delay_start;
                    else
                    {
                        y_delay = y_atTrajStart + (radius/2)*sin(2*rotvel*(traj_time - pos_pub_delay*sampleTime));
//                        std::cout<<"y_delay_started gets true"<<"\n";
                        y_delay_started = true;
                    }
                else
                    y_delay = y_atTrajStart + (radius/2)*sin(2*rotvel*(traj_time - pos_pub_delay*sampleTime));

                z = z_atTrajStart + del_z*(sin(rotvel*traj_time)) * const_z;
                if (!z_delay_started)
                    if (std::abs(z - z_delay_start) < del_z*(sin(rotvel*(pos_pub_delay*sampleTime))) * const_z)
                        z_delay = z_delay_start;
                    else
                    {
                        z_delay = z_atTrajStart + del_z*(sin(rotvel*(traj_time - pos_pub_delay*sampleTime))) * const_z;
//                        std::cout<<"z_delay_started gets true"<<"\n";
                        z_delay_started = true;
                    }
                else
                    z_delay = z_atTrajStart + del_z*(sin(rotvel*(traj_time - pos_pub_delay*sampleTime))) * const_z;
            }

            if(traj_type == 5) // square (TO BE IMPROVED for delay)
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
                if (std::abs(sin(rotvel*traj_time)-1) < 0.001 || std::abs(sin(rotvel*traj_time)+1) < 0.001)
                    x = x_atTrajStart + radius*(sin(rotvel*traj_time)<0 ? std::floor(sin(rotvel*traj_time)) : std::ceil(sin(rotvel*traj_time)));
                else
                    x = x_last;
                if(traj_time < 0.25*time_period)
                {
                    y = y_atTrajStart;
                    z = z_atTrajStart;
                    t_last_z = ros::Time::now().toSec();
                }
                else
                {
                    traj_time_z = t - t_last_z;
                    if (std::abs(cos(rotvel*traj_time)-1) < 0.001 || std::abs(cos(rotvel*traj_time)+1) < 0.001)
                        y = y_atTrajStart + radius*(cos(rotvel*traj_time)<0 ? std::floor(cos(rotvel*traj_time)) : std::ceil(cos(rotvel*traj_time)));
                    else
                        y = y_last;
                    if (std::abs(sin(rotvel*traj_time_z)-1) < 0.001 || std::abs(sin(rotvel*traj_time_z)+1) < 0.001)
                        z = z_atTrajStart - del_z*(sin(rotvel*traj_time_z)<0 ? std::floor(sin(rotvel*traj_time_z)) : std::ceil(sin(rotvel*traj_time_z))) * const_z;
                    else
                        z = z_last;
                }
                x_delay = x;
                y_delay = y;
                z_delay = z;

            }
            if(traj_type == 6) // setpoint (TO BE IMPROVED for delay)
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

                    x = x_sp_start;
                    y = y_sp_start;
                    z = z_sp_start;

                    R_BI.setRotation(setpoint_att_quat);
                    R_BI = R_BI.transpose();
                    R_IB = R_BI.transpose();

                    std::cout<<"R_BI = ";
                    for (int i=0; i<3; i++)
                    {
                        for (int j=0; j<3; j++)
                            std::cout<<R_BI[i][j]<<", ";
                        std::cout<<"\n";
                    }
                    std::cout<<"R_IB = ";
                    for (int i=0; i<3; i++)
                    {
                        for (int j=0; j<3; j++)
                            std::cout<<R_IB[i][j]<<", ";
                        std::cout<<"\n";
                    }

                    x_B_sp_start = R_BI[0][0]*x + R_BI[0][1]*y;
                    y_B_sp_start = R_BI[1][0]*x + R_BI[1][1]*y;
                    x_B = x_B_sp_start;
                    y_B = y_B_sp_start;

                    x_B_sp_end = R_BI[0][0]*x_sp_end + R_BI[0][1]*y_sp_end;
                    y_B_sp_end = R_BI[1][0]*x_sp_end + R_BI[1][1]*y_sp_end;


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
                    num_turns = (int)(std::abs(z_sp_start-z_sp_end)/z_interval_dist);
                    if (num_turns % 2 != 0)
                        z_interval_dist_updated = std::abs(z_sp_start-z_sp_end)/++num_turns;
                    else
                        z_interval_dist_updated = z_interval_dist;
                    sp_left_corner_reached_flag = true;
                    sp_right_corner_reached_flag = false;
                    sp_z_counter = 0;
                    sp_z_counter_switch = true;

                    if (!lidar_start)
                    {
                        ROS_WARN("Lidar is not started!");
                        ROS_WARN("Holding X_B = %f!",x_B_sp_start);
//                        if (wall_direc == 1)
//                            ROS_WARN("Holding X = %f!",x_sp_start);
//                        else if (wall_direc == 2)
//                            ROS_WARN("Holding Y = %f!",y_sp_start);
                    }

                    std::cout<<"wall_direc = "<<wall_direc<<"\n";
                    std::cout<<"num_turns = "<<num_turns<<"\n";
                    std::cout<<"z_interval_dist = "<<z_interval_dist<<"\n";
                    std::cout<<"z_interval_dist_updated = "<<z_interval_dist_updated<<"\n";


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

                if (lidar_start)
//                    x_B = x_B_sp_start + (lidar_read_data-wall_dist)
//                                       + (R_BI[0][0]*current_pos[0] +
//                                          R_BI[0][1]*current_pos[1]);
                {
                    // TO BE: removed
                    if (use_current_pos)
                        x_B = x_B_sp_start + (lidar_read_data-wall_dist)
                                           + (R_BI[0][0]*current_pos[0] +
                                              R_BI[0][1]*current_pos[1]);
                    else
                        x_B = x_B_sp_start + (lidar_read_data-wall_dist);
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
                            std::cout<<"sp_z_counter = "<<sp_z_counter++<<"\n";
                            sp_z_counter_switch = false;
                            t_last_eachRun = ros::Time::now().toSec();
                        }
                        y_B -= absvel*sampleTime;
                        z = z;
                        if (std::abs(y_B-y_B_sp_end)<=absvel*sampleTime &&
                            std::abs(traj_time_eachRun-std::abs(y_B_sp_start-y_B_sp_end)/absvel<1))
                        {
                            std::cout<<"right corner reached!\n";
                            sp_left_corner_reached_flag = false;
                            sp_right_corner_reached_flag = false;
                        }

                    }
                    else if ((y_B < y_B_sp_start) && !sp_left_corner_reached_flag && sp_right_corner_reached_flag)
                    {
                        if (sp_z_counter_switch)
                        {
                            std::cout<<"sp_z_counter = "<<sp_z_counter++<<"\n";
                            sp_z_counter_switch = false;
                            t_last_eachRun = ros::Time::now().toSec();
                        }
                        y_B += absvel*sampleTime;
                        z = z;
                        if (std::abs(y_B-y_B_sp_start)<=absvel*sampleTime &&
                            std::abs(traj_time_eachRun-std::abs(y_B_sp_start-y_B_sp_end)/absvel<1))
                        {
                            std::cout<<"left corner reached!\n";
                            sp_left_corner_reached_flag = false;
                            sp_right_corner_reached_flag = false;
                        }
                    }
                    else if ((!sp_left_corner_reached_flag && !sp_right_corner_reached_flag) &&
                             z>(z_sp_start-z_interval_dist_updated*sp_z_counter))
                    {
                        if (!sp_z_counter_switch)
                        {
                            sp_z_counter_switch = true;
                            t_last_eachRun = ros::Time::now().toSec();
                        }
                        y_B = y_B;
                        z -= absvel*sampleTime;
                        if (std::abs(z-(z_sp_start-z_interval_dist_updated*sp_z_counter))<=absvel*sampleTime)
                        {
                            if (std::abs(y_B-y_B_sp_end)<=absvel*sampleTime)
                            {
                                sp_left_corner_reached_flag = false;
                                sp_right_corner_reached_flag = true;
                            }
                            else if (std::abs(y_B-y_B_sp_start)<=absvel*sampleTime)
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
                        std::cout<<"Final setpoint reached!\n";
                        std::cout<<"Trajectory complete!\n";
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

                x = R_IB[0][0]*x_B + R_IB[0][1]*y_B;
                y = R_IB[1][0]*x_B + R_IB[1][1]*y_B;

                x_delay = x;
                y_delay = y;
                z_delay = z;
            }
            ref_yaw.data = deg2rad*yaw_hover;
            setpoint_att_quat.setRPY(0,0,ref_yaw.data);
        }
        else if(traj_start == 1 || land_flag)
        {
            if (print_flag_traj_start == 0)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Holding position at [x,y,z]: %.2f,%.2f,%.2f",x,y,z);
                print_flag_traj_start = 1;
            }
            x = x;
            y = y;
            z = z;

            x_delay = x;
            y_delay = y;
            z_delay = z;

            if (land_flag)
                ref_yaw.data = ref_yaw.data;
            else
                ref_yaw.data = deg2rad*yaw_hover;
            setpoint_att_quat.setRPY(0,0,ref_yaw.data);

            t_last = ros::Time::now().toSec();
        }
        else
        {
            if(traj_started_flag)
            {
//                pos_ref_start_msg.pose.position.z = 0.0;
                traj_started_flag = false;
            }

            if (print_flag_traj_start == 0)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Default reference position only!");
                ROS_INFO("Position reference x =  %.2f m!",pos_ref_start_msg.pose.position.x);
                ROS_INFO("Position reference y =  %.2f m!",pos_ref_start_msg.pose.position.y);
                ROS_INFO("Position reference z =  %.2f m!",pos_ref_start_msg.pose.position.z);
                print_flag_traj_start = 1;
            }

            x = pos_ref_start_msg.pose.position.x;
            y = pos_ref_start_msg.pose.position.y;
            z = pos_ref_start_msg.pose.position.z;

            x_delay = x;
            y_delay = y;
            z_delay = z;

            ref_yaw.data = ref_yaw.data;
            setpoint_att_quat.setRPY(0,0,ref_yaw.data);

            t_last = ros::Time::now().toSec();
//            std::cout<<"t_last = "<<t_last<<"\n";

            climbed_flag = false;
            landed_flag = false;
        }

        if(land_flag)
        {
            while(ros::ok() && land_flag && !landed_flag && z_delay > 0.0)
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
                z = z > 0 ? z - land_rate*sampleTime : 0;

                x_delay = x;
                y_delay = y;
                z_delay = std::abs(z - z_delay_start) < land_rate*pos_pub_delay*sampleTime ? z_delay_start : z_delay - land_rate*sampleTime;

                u = (x - x_last)/sampleTime;
                v = (y - y_last)/sampleTime;
                w = (z - z_last)/sampleTime;

                ref_yaw.data = deg2rad*yaw_hover;

                if(!pub_setpoint_pos)
                {
                    reftrajectory_msg.x = x;
                    reftrajectory_msg.y = y;
                    reftrajectory_msg.z = z;
                    ref_pos_pub.publish(reftrajectory_msg);
                    reftrajectory_delay_msg.x = x_delay;
                    reftrajectory_delay_msg.y = y_delay;
                    reftrajectory_delay_msg.z = z_delay;
                    ref_pos_delay_pub.publish(reftrajectory_delay_msg);

                    reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
                    reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
                    reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);
                    ref_vel_pub.publish(reftrajectory_vel_msg);

                    ref_yaw_pub.publish(ref_yaw);
                }
                else
                {
                    setpoint_pos_msg.header.stamp = ros::Time::now();
                    setpoint_pos_msg.pose.position.x = x;
                    setpoint_pos_msg.pose.position.y = y;
                    setpoint_pos_msg.pose.position.z = z;

                    setpoint_att_quat.setRPY(0,0,ref_yaw.data);
                    setpoint_pos_msg.pose.orientation.x = setpoint_att_quat.getX();
                    setpoint_pos_msg.pose.orientation.y = setpoint_att_quat.getY();
                    setpoint_pos_msg.pose.orientation.z = setpoint_att_quat.getZ();
                    setpoint_pos_msg.pose.orientation.w = setpoint_att_quat.getW();
                    setpoint_pos_pub.publish(setpoint_pos_msg);
                }

                if(z_delay <= 0.0)
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

                traj_on_pub.publish(traj_start_msg);
                ros::spinOnce();
                rate.sleep();
            }
        }

        u = (x - x_last)/sampleTime;
        v = (y - y_last)/sampleTime;
        w = (z - z_last)/sampleTime;

//        std::cout<<"absolute vel along x & y = "<<sqrt(u*u + v*v)<<"\n";

        x_last = x;
        y_last = y;
        z_last = z;

        lidar_read_filtered_pub.publish(lidar_read_filtered_msg);
        if(!pub_setpoint_pos)
        {
            reftrajectory_msg.x = x;
            reftrajectory_msg.y = y;
            reftrajectory_msg.z = z;
            ref_pos_pub.publish(reftrajectory_msg);
            reftrajectory_delay_msg.x = x_delay;
            reftrajectory_delay_msg.y = y_delay;
            reftrajectory_delay_msg.z = z_delay;
            ref_pos_delay_pub.publish(reftrajectory_delay_msg);

            reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
            reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
            reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);
            ref_vel_pub.publish(reftrajectory_vel_msg);

            ref_yaw_pub.publish(ref_yaw);
        }
        else
        {
            setpoint_pos_msg.header.stamp = ros::Time::now();
            setpoint_pos_msg.pose.position.x = x;
            setpoint_pos_msg.pose.position.y = y;
            setpoint_pos_msg.pose.position.z = z;

            setpoint_att_quat.setRPY(0,0,ref_yaw.data);
            setpoint_pos_msg.pose.orientation.x = setpoint_att_quat.getX();
            setpoint_pos_msg.pose.orientation.y = setpoint_att_quat.getY();
            setpoint_pos_msg.pose.orientation.z = setpoint_att_quat.getZ();
            setpoint_pos_msg.pose.orientation.w = setpoint_att_quat.getW();
            setpoint_pos_pub.publish(setpoint_pos_msg);
        }

        traj_on_pub.publish(traj_start_msg);

        ros::spinOnce();
        rate.sleep();

    }

    return 0;

}
