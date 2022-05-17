/**
 * @file   dummyLidar.cpp
 * @author Mohit Mehndiratta
 * @date   April 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Range.h>
#include <dynamic_reconfigure/server.h>
#include <dji_m100_trajectory/set_dummyLidarConfig.h>

using namespace std;

double sampleTime = 0.01;
struct _lidar
{
    bool send_nan;
    double range, min_range, max_range;
} Llidar, Clidar, Rlidar, sonar;
void dynamicReconfigureCallback(dji_m100_trajectory::set_dummyLidarConfig &config, uint32_t level)
{
    Llidar.send_nan = config.Llidar_nan;
    Llidar.range = config.Llidar_range;
    Llidar.min_range = config.Llidar_min_range;
    Llidar.max_range = config.Llidar_max_range;
    Clidar.send_nan = config.Clidar_nan;
    Clidar.range = config.Clidar_range;
    Clidar.min_range = config.Clidar_min_range;
    Clidar.max_range = config.Clidar_max_range;
    Rlidar.send_nan = config.Rlidar_nan;
    Rlidar.range = config.Rlidar_range;
    Rlidar.min_range = config.Rlidar_min_range;
    Rlidar.max_range = config.Rlidar_max_range;
    sonar.send_nan = config.sonar_nan;
    sonar.range = config.sonar_range;
    sonar.min_range = config.sonar_min_range;
    sonar.max_range = config.sonar_max_range;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummyLidar");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<dji_m100_trajectory::set_dummyLidarConfig> server;
    dynamic_reconfigure::Server<dji_m100_trajectory::set_dummyLidarConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    std::string Llidar_topic, Clidar_topic, Rlidar_topic, sonar_topic;
    ros::param::get("Llidar_topic", Llidar_topic);
    ros::param::get("Clidar_topic", Clidar_topic);
    ros::param::get("Rlidar_topic", Rlidar_topic);
    ros::param::get("sonar_topic", sonar_topic);

    ros::Publisher Llidar_read_pub = nh.advertise<sensor_msgs::Range>(Llidar_topic, 1, true);
    ros::Publisher Clidar_read_pub = nh.advertise<sensor_msgs::Range>(Clidar_topic, 1, true);
    ros::Publisher Rlidar_read_pub = nh.advertise<sensor_msgs::Range>(Rlidar_topic, 1, true);
    ros::Publisher sonar_read_pub = nh.advertise<sensor_msgs::Range>(sonar_topic, 1, true);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1/sampleTime);

    sensor_msgs::Range lidar_read_msg, sonar_read_msg;

    while(ros::ok())
    {
        lidar_read_msg.header.frame_id = "0";
        lidar_read_msg.header.stamp = ros::Time::now();
        lidar_read_msg.max_range = Llidar.max_range;
        lidar_read_msg.min_range = Llidar.min_range;
        if (Llidar.send_nan)
            lidar_read_msg.range = std::numeric_limits<float>::quiet_NaN();
        else
            lidar_read_msg.range = Llidar.range;
        Llidar_read_pub.publish(lidar_read_msg);

        lidar_read_msg.header.frame_id = "0";
        lidar_read_msg.header.stamp = ros::Time::now();
        lidar_read_msg.max_range = Clidar.max_range;
        lidar_read_msg.min_range = Clidar.min_range;
        if (Clidar.send_nan)
            lidar_read_msg.range = std::numeric_limits<float>::quiet_NaN();
        else
            lidar_read_msg.range = Clidar.range;
        Clidar_read_pub.publish(lidar_read_msg);

        lidar_read_msg.header.frame_id = "0";
        lidar_read_msg.header.stamp = ros::Time::now();
        lidar_read_msg.max_range = Rlidar.max_range;
        lidar_read_msg.min_range = Rlidar.min_range;
        if (Rlidar.send_nan)
            lidar_read_msg.range = std::numeric_limits<float>::quiet_NaN();
        else
            lidar_read_msg.range = Rlidar.range;
        Rlidar_read_pub.publish(lidar_read_msg);

        sonar_read_msg.header.frame_id = "0";
        sonar_read_msg.header.stamp = ros::Time::now();
        sonar_read_msg.max_range = sonar.max_range;
        sonar_read_msg.min_range = sonar.min_range;
        if (sonar.send_nan)
            sonar_read_msg.range = std::numeric_limits<float>::quiet_NaN();
        else
            sonar_read_msg.range = sonar.range;
        sonar_read_pub.publish(sonar_read_msg);

        ros::spinOnce();
        rate.sleep();
    }
  return 0;
}
