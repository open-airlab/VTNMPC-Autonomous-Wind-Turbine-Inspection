#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>

using namespace std;

double sample_time = 1.0 / 50.0;;

nav_msgs::Odometry airsim_odom;
geometry_msgs::PoseStamped mocap_pos_in, mocap_pos_out;

void airsim_pos_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    airsim_odom = *msg;

    mocap_pos_in.header.stamp = ros::Time::now();
    mocap_pos_in.header.frame_id = "";

    mocap_pos_in.pose.position.x = airsim_odom.pose.pose.position.x;
    mocap_pos_in.pose.position.y = airsim_odom.pose.pose.position.y;
    mocap_pos_in.pose.position.z = airsim_odom.pose.pose.position.z;

    mocap_pos_in.pose.orientation.x = airsim_odom.pose.pose.orientation.x;
    mocap_pos_in.pose.orientation.y = airsim_odom.pose.pose.orientation.y;
    mocap_pos_in.pose.orientation.z = airsim_odom.pose.pose.orientation.z;
    mocap_pos_in.pose.orientation.w = airsim_odom.pose.pose.orientation.w;
}
//
//std::vector<int> omegas_idx;
//std::vector<double> omegas_vec;
//gazebo_msgs::LinkStates gazebo_linkpos;

//void gazebo_linkpos_cb(const gazebo_msgs::LinkStates::ConstPtr &msg) {
//    gazebo_linkpos = *msg;
//}

geometry_msgs::PoseStamped local_pos;

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    local_pos = *msg;
}

geometry_msgs::TwistStamped local_vel, mocap_vel, mocap_vel_body;

void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    local_vel = *msg;
}

double mean_filter(double val, std::vector<double> &data_unfiltered) {
    // erase from the front!
    data_unfiltered.erase(data_unfiltered.begin());
    data_unfiltered.push_back(val);

    double new_val = 0.0;
    for (double i : data_unfiltered)
        new_val += i;
    new_val = new_val / data_unfiltered.size();

    return new_val;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mocap_bridge_gazebo");
    ros::NodeHandle nh;

    // ros::Subscriber pos_sub = nh.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 10, pos_cb);
    ros::Subscriber local_pos_sub =
            nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber local_vel_sub =
            nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body", 10, local_vel_cb);
    ros::Subscriber airsim_pos_sub = nh.subscribe<nav_msgs::Odometry>("airsim_node/m100/ground_truth_pose_enu", 10, airsim_pos_cb);
//    ros::Subscriber gazebo_linkpos_sub =
//        nh.subscribe<gazebo_msgs::LinkStates>("gazebo/link_states", 10, gazebo_linkpos_cb);

    ros::Publisher pos_pub_mocap = nh.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 1);
    //    ros::Publisher pos_pub_mocap = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);
    ros::Publisher vel_pub_mocap = nh.advertise<geometry_msgs::TwistStamped>("mavros/mocap/velocity", 1);
    ros::Publisher vel_body_pub_mocap = nh.advertise<geometry_msgs::TwistStamped>("mavros/mocap/velocity_body", 1);
//    ros::Publisher omegas_pub = nh.advertise<std_msgs::Float64MultiArray>("mavros/omegas", 1, true);
    ros::Rate rate(1 / sample_time);

    double roll_out, pitch_out, yaw_out, pi = 22 / 7;
    double roll_in, pitch_in, yaw_in;
    //    double roll_current, pitch_current, yaw_current;
    double roll_local, pitch_local, yaw_local;

    std::vector<double> mocap_vel_x_unfiltered(3, 0.0);
    std::vector<double> mocap_vel_y_unfiltered(3, 0.0);
    std::vector<double> mocap_vel_z_unfiltered(3, 0.0);

    double mocap_pos_x_last = 0.0;
    double mocap_pos_y_last = 0.0;
    double mocap_pos_z_last = 0.0;

    //    omegas_idx.resize(4, 0);

    // To obtain "rotor_" indexes
    for (int i = 0; i < (int) (1 / sample_time); ++i) {
        ros::spinOnce();
        rate.sleep();
    }
//    for (unsigned int i = 0; i < gazebo_linkpos.name.size(); i++) {
//        std::string link_name = gazebo_linkpos.name[i];
//        int pos = link_name.find("rotor_");
//        if (pos != link_name.npos) {
//            omegas_idx.push_back(i);
//            //            std::cout<<link_name.c_str()<<" found at "<<i<<"\n";
//        }
//    }
    //    std::cout<<"omegas_idx.size() = "<<omegas_idx.size()<<"\n";
//    omegas_vec.resize(omegas_idx.size(), 0.0);

    while (ros::ok()) {
        mocap_pos_out.header.stamp = ros::Time::now();
        mocap_pos_out.header.frame_id = "";

        mocap_pos_out.pose.position.x = mocap_pos_in.pose.position.x;
        mocap_pos_out.pose.position.y = mocap_pos_in.pose.position.y;
        mocap_pos_out.pose.position.z = mocap_pos_in.pose.position.z;

        tf::Quaternion mocap_q_in(mocap_pos_in.pose.orientation.x,
                                  mocap_pos_in.pose.orientation.y,
                                  mocap_pos_in.pose.orientation.z,
                                  mocap_pos_in.pose.orientation.w);
        tf::Matrix3x3 mocap_m_in(mocap_q_in);
        mocap_m_in.getRPY(roll_in, pitch_in, yaw_in);

        tf::Quaternion mocap_q_out(mocap_pos_out.pose.orientation.x,
                                   mocap_pos_out.pose.orientation.y,
                                   mocap_pos_out.pose.orientation.z,
                                   mocap_pos_out.pose.orientation.w);
        tf::Matrix3x3 mocap_m_out(mocap_q_out);
        mocap_m_out.getRPY(roll_out, pitch_out, yaw_out);

        roll_out = roll_in;
        pitch_out = pitch_in;
        yaw_out = yaw_in;

        mocap_q_out.setRPY(roll_out, pitch_out, yaw_out);

        mocap_pos_out.pose.orientation.x = mocap_q_out.getX();
        mocap_pos_out.pose.orientation.y = mocap_q_out.getY();
        mocap_pos_out.pose.orientation.z = mocap_q_out.getZ();
        mocap_pos_out.pose.orientation.w = mocap_q_out.getW();

        pos_pub_mocap.publish(mocap_pos_out);

        tf::Quaternion q_local(local_pos.pose.orientation.x,
                               local_pos.pose.orientation.y,
                               local_pos.pose.orientation.z,
                               local_pos.pose.orientation.w);
        tf::Matrix3x3 m_local(q_local);
        m_local.getRPY(roll_local, pitch_local, yaw_local);

        // ROS_INFO("mocap Roll = %f deg",roll_out*(180/pi));
        // ROS_INFO("mocap Pitch = %f deg",pitch_out*(180/pi));
        // ROS_INFO("mocap Yaw = %f deg",yaw_out*(180/pi));
        // ROS_INFO("----------------------------------------");
        // ROS_INFO("Local Roll = %f deg",roll_local*(180/pi));
        // ROS_INFO("Local Pitch = %f deg",pitch_local*(180/pi));
        // ROS_INFO("Local Yaw = %f deg",yaw_local*(180/pi));
        // ROS_INFO("----------------------------------------");
        // ROS_INFO("----------------------------------------");

        //        ROS_INFO("mocap X = %f",mocap_pos_out.pose.position.x);
        //        ROS_INFO("mocap Y = %f",mocap_pos_out.pose.position.y);
        //        ROS_INFO("mocap Z = %f",mocap_pos_out.pose.position.z);
        //        ROS_INFO("----------------------------------------");
        //        ROS_INFO("Local X = %f",local_pos.pose.position.x);
        //        ROS_INFO("Local Y = %f",local_pos.pose.position.y);
        //        ROS_INFO("Local Z = %f",local_pos.pose.position.z);
        //        ROS_INFO("----------------------------------------");
        //        ROS_INFO("----------------------------------------");

        std::cout << "mocap [X, Y, Z] = " << mocap_pos_out.pose.position.x << ", " << mocap_pos_out.pose.position.y
                  << ", " << mocap_pos_out.pose.position.z << "\n";
        std::cout << "----------------------------------------"
                  << "\n";
        std::cout << "Local [X, Y, Z] = " << local_pos.pose.position.x << ", " << local_pos.pose.position.y << ", "
                  << local_pos.pose.position.z << "\n";
        std::cout << "----------------------------------------"
                  << "\n";

        mocap_vel.header.stamp = ros::Time::now();

        mocap_vel.twist.linear.x = (mocap_pos_out.pose.position.x - mocap_pos_x_last) / sample_time;
        mocap_vel.twist.linear.y = (mocap_pos_out.pose.position.y - mocap_pos_y_last) / sample_time;
        mocap_vel.twist.linear.z = (mocap_pos_out.pose.position.z - mocap_pos_z_last) / sample_time;

        // With filter
        mocap_vel.twist.linear.x =
                mean_filter((mocap_pos_out.pose.position.x - mocap_pos_x_last) / sample_time, mocap_vel_x_unfiltered);
        mocap_vel.twist.linear.y =
                mean_filter((mocap_pos_out.pose.position.y - mocap_pos_y_last) / sample_time, mocap_vel_y_unfiltered);
        mocap_vel.twist.linear.z =
                mean_filter((mocap_pos_out.pose.position.z - mocap_pos_z_last) / sample_time, mocap_vel_z_unfiltered);

        // this is the angular velocity in body frame
        // TOBE: change it to world frame
        mocap_vel.twist.angular = local_vel.twist.angular;

        vel_pub_mocap.publish(mocap_vel);

        // velocity in body frame
        tf::Matrix3x3 R_BI(mocap_q_in);
        R_BI = R_BI.transpose();

        mocap_vel_body.header.stamp = ros::Time::now();
        mocap_vel_body.twist.linear.x = R_BI[0][0] * mocap_vel.twist.linear.x + R_BI[0][1] * mocap_vel.twist.linear.y +
                                        R_BI[0][2] * mocap_vel.twist.linear.z;
        mocap_vel_body.twist.linear.y = R_BI[1][0] * mocap_vel.twist.linear.x + R_BI[1][1] * mocap_vel.twist.linear.y +
                                        R_BI[1][2] * mocap_vel.twist.linear.z;
        mocap_vel_body.twist.linear.z = R_BI[2][0] * mocap_vel.twist.linear.x + R_BI[2][1] * mocap_vel.twist.linear.y +
                                        R_BI[2][2] * mocap_vel.twist.linear.z;
        mocap_vel_body.twist.angular = local_vel.twist.angular;
        vel_body_pub_mocap.publish(mocap_vel_body);

        mocap_pos_x_last = mocap_pos_out.pose.position.x;
        mocap_pos_y_last = mocap_pos_out.pose.position.y;
        mocap_pos_z_last = mocap_pos_out.pose.position.z;

//        std::cout << "omegas_vec = ";
//        for (int i = 0; i < omegas_idx.size(); i++) {
//            omegas_vec[i] = 10 * std::abs(gazebo_linkpos.twist[omegas_idx[i]].angular.z);
//            std::cout << omegas_vec[i] << ",";
//        }
//        std::cout << "\n";

//        std_msgs::Float64MultiArray omegas_msg;
//        omegas_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
//        omegas_msg.layout.dim[0].size = omegas_vec.size();
//        omegas_msg.layout.dim[0].stride = 1;
//        omegas_msg.layout.dim[0].label = "omegas 1, 2, 3, 4 (rad/s)";
//        omegas_msg.data.clear();
//        omegas_msg.data.insert(omegas_msg.data.end(), omegas_vec.begin(), omegas_vec.end());
//        omegas_pub.publish(omegas_msg);

        std::cout << "----------------------------------------"
                  << "\n";
        std::cout << "----------------------------------------"
                  << "\n";

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
