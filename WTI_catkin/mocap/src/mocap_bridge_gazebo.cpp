#include <mocap/mocap_gazebo.h>

using namespace std;
double sample_time = 0.01;

void gazebo_modelpos_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    gazebo_modelpos = *msg;
}

void gazebo_linkpos_cb(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    gazebo_linkpos = *msg;
}

double mean_filter(double val, std::vector<double>& data_unfiltered)
{
    // erase from the front!
    data_unfiltered.erase(data_unfiltered.begin());
    data_unfiltered.push_back(val);

    double new_val = 0.0;
    for (int i = 0; i < data_unfiltered.size(); ++i)
        new_val += data_unfiltered[i];
    new_val = new_val / (data_unfiltered.size());

    return new_val;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mocap_bridge_gazebo");
    ros::NodeHandle nh;

    gazebo_modelpos_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, gazebo_modelpos_cb);
    gazebo_linkpos_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, gazebo_linkpos_cb);

    pos_mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 1);
    vel_mocap_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/mocap/velocity", 1);
    vel_body_mocap_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/mocap/velocity_body", 1);
    omegas_pub = nh.advertise<std_msgs::Float64MultiArray>("/mavros/mocap/omegas", 1, true);
    ros::Rate rate(1 / sample_time);

    filter_size = 3;

    mocap_vel_x_unfiltered.resize(filter_size, 0.0);
    mocap_vel_y_unfiltered.resize(filter_size, 0.0);
    mocap_vel_z_unfiltered.resize(filter_size, 0.0);

    // To obtain model and omegas indeces
    for (int i = 0; i < (int)(1 / sample_time); ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }
    for (unsigned int i = 0; i < gazebo_modelpos.name.size(); i++)
    {
        std::string link_name = gazebo_modelpos.name[i];
        int pos = link_name.find("matrice_100");
        if (pos != link_name.npos)
        {
            model_idx = i;
            std::cout << link_name.c_str() << " found at " << i << "\n";
        }
    }
    for (unsigned int i = 0; i < gazebo_linkpos.name.size(); i++)
    {
        std::string link_name = gazebo_linkpos.name[i];
        int pos = link_name.find("rotor_");
        if (pos != link_name.npos)
        {
            omegas_idx.push_back(i);
            std::cout << link_name.c_str() << " found at " << i << "\n";
        }
    }
    //    std::cout<<"omegas_idx.size() = "<<omegas_idx.size()<<"\n";
    omegas_vec.resize(omegas_idx.size(), 0.0);

    while (ros::ok())
    {
        // mocap pos data
        mocap_pos_out.header.stamp = ros::Time::now();
        mocap_pos_out.header.frame_id = "";

        mocap_pos_out.pose.position.x = gazebo_modelpos.pose[model_idx].position.x;
        mocap_pos_out.pose.position.y = gazebo_modelpos.pose[model_idx].position.y;
        mocap_pos_out.pose.position.z = gazebo_modelpos.pose[model_idx].position.z;

        mocap_pos_out.pose.orientation.x = gazebo_modelpos.pose[model_idx].orientation.x;
        mocap_pos_out.pose.orientation.y = gazebo_modelpos.pose[model_idx].orientation.y;
        mocap_pos_out.pose.orientation.z = gazebo_modelpos.pose[model_idx].orientation.z;
        mocap_pos_out.pose.orientation.w = gazebo_modelpos.pose[model_idx].orientation.w;

        pos_mocap_pub.publish(mocap_pos_out);

        tf::Quaternion mocap_q(mocap_pos_out.pose.orientation.x,
                               mocap_pos_out.pose.orientation.y,
                               mocap_pos_out.pose.orientation.z,
                               mocap_pos_out.pose.orientation.w);
        tf::Matrix3x3 mocap_m(mocap_q);
        mocap_m.getRPY(roll_out, pitch_out, yaw_out);

        std::cout << "(X, Y, Z) [m] = " << mocap_pos_out.pose.position.x << ", " << mocap_pos_out.pose.position.y
                  << ", " << mocap_pos_out.pose.position.z << "\n";
        //        std::cout<<"----------------------------------------"<<"\n";
        std::cout << "(roll, pitch, yaw) [deg] = " << roll_out * (180 / M_PI) << ", " << pitch_out * (180 / M_PI)
                  << ", " << yaw_out * (180 / M_PI) << "\n";
        //        std::cout<<"----------------------------------------"<<"\n";

        // mocap velocity data
        mocap_vel.header.stamp = ros::Time::now();
        // With filter
        mocap_vel.twist.linear.x = mean_filter(gazebo_modelpos.twist[model_idx].linear.x, mocap_vel_x_unfiltered);
        mocap_vel.twist.linear.y = mean_filter(gazebo_modelpos.twist[model_idx].linear.y, mocap_vel_y_unfiltered);
        mocap_vel.twist.linear.z = mean_filter(gazebo_modelpos.twist[model_idx].linear.z, mocap_vel_z_unfiltered);

        // this is the angular velocity in body frame
        // TOBE: change it to world frame
        mocap_vel.twist.angular = gazebo_modelpos.twist[model_idx].angular;

        vel_mocap_pub.publish(mocap_vel);

        // mocap velocity data in body frame
        tf::Matrix3x3 R_BI(mocap_q);
        R_BI = R_BI.transpose();

        mocap_vel_body.header.stamp = ros::Time::now();
        mocap_vel_body.twist.linear.x = R_BI[0][0] * mocap_vel.twist.linear.x + R_BI[0][1] * mocap_vel.twist.linear.y +
                                        R_BI[0][2] * mocap_vel.twist.linear.z;
        mocap_vel_body.twist.linear.y = R_BI[1][0] * mocap_vel.twist.linear.x + R_BI[1][1] * mocap_vel.twist.linear.y +
                                        R_BI[1][2] * mocap_vel.twist.linear.z;
        mocap_vel_body.twist.linear.z = R_BI[2][0] * mocap_vel.twist.linear.x + R_BI[2][1] * mocap_vel.twist.linear.y +
                                        R_BI[2][2] * mocap_vel.twist.linear.z;
        mocap_vel_body.twist.angular = gazebo_modelpos.twist[model_idx].angular;
        vel_body_mocap_pub.publish(mocap_vel_body);

        std::cout << "(Vx, Vy, Vz) [m/s] = " << mocap_vel.twist.linear.x << ", " << mocap_vel.twist.linear.y << ", "
                  << mocap_vel.twist.linear.z << "\n";
        //        std::cout<<"----------------------------------------"<<"\n";
        std::cout << "(Vx_B, Vy_B, Vz_B) [m/s] = " << mocap_vel_body.twist.linear.x << ", "
                  << mocap_vel_body.twist.linear.y << ", " << mocap_vel_body.twist.linear.z << "\n";
        //        std::cout<<"----------------------------------------"<<"\n";
        std::cout << "(p_rate, q_rate, r_rate) [deg/s] = " << mocap_vel.twist.angular.x * (180 / M_PI) << ", "
                  << mocap_vel.twist.angular.y * (180 / M_PI) << ", " << mocap_vel.twist.angular.z * (180 / M_PI)
                  << "\n";
        //        std::cout<<"----------------------------------------"<<"\n";

        std::cout << "(w_fl, w_fr, w_rl, w_rr) [rad/s] = ";
        for (int i = 0; i < omegas_idx.size(); i++)
        {
            omegas_vec[i] = 10 * std::abs(gazebo_linkpos.twist[omegas_idx[i]].angular.z);
            std::cout << omegas_vec[i] << ",";
        }
        std::cout << "\n";
        std_msgs::Float64MultiArray omegas_msg;
        omegas_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        omegas_msg.layout.dim[0].size = omegas_vec.size();
        omegas_msg.layout.dim[0].stride = 1;
        omegas_msg.layout.dim[0].label = "w_1, w_2, w_3, w_4 (rad/s)";
        omegas_msg.data.clear();
        omegas_msg.data.insert(omegas_msg.data.end(), omegas_vec.begin(), omegas_vec.end());
        omegas_pub.publish(omegas_msg);

        std::cout << "----------------------------------------"
                  << "\n";
        std::cout << "----------------------------------------"
                  << "\n";

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
