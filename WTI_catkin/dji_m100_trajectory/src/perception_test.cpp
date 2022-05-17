#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <airsim_ros_wrapper.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <ros/spinner.h>
#include "std_msgs/Float64.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
//#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <cmath>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>

#include <nav_msgs/Odometry.h>
//using namespace std
#include "geometry_msgs/TwistStamped.h"

float px, py, nx, ny;

std::vector<double>  px_vec, py_vec,nx_vec,ny_vec;
std_msgs::Float64 px_val,py_val,nx_val,ny_val;
int main(int argc, char **argv)
{



    std::ifstream inputFilepx("/home/airlab/hakim_ws/src/WTI_catkin/dji_m100_trajectory/src/perception_test/px.txt");
    std::ifstream inputFilepy("/home/airlab/hakim_ws/src/WTI_catkin/dji_m100_trajectory/src/perception_test/py.txt");
    std::ifstream inputFilenx("/home/airlab/hakim_ws/src/WTI_catkin/dji_m100_trajectory/src/perception_test/nx.txt");
    std::ifstream inputFileny("/home/airlab/hakim_ws/src/WTI_catkin/dji_m100_trajectory/src/perception_test/ny.txt");


    while (inputFilepx >> px)
    {
    px_vec.push_back(px);
    }
   
    while (inputFilepy >> py)
    {
    py_vec.push_back(py);
    }

    while (inputFilenx >> nx)
    {
    nx_vec.push_back(nx);
    }
   
    while (inputFileny >> ny)
    {
    ny_vec.push_back(ny);
    }





  ros::init(argc, argv, "perception_test");


  ros::NodeHandle n;

  ros::Publisher px_pub = n.advertise<std_msgs::Float64>("px", 50);
  ros::Publisher py_pub = n.advertise<std_msgs::Float64>("py", 50);
  ros::Publisher nx_pub = n.advertise<std_msgs::Float64>("nx", 50);
  ros::Publisher ny_pub = n.advertise<std_msgs::Float64>("ny", 50);

  ros::Rate loop_rate(0.1);

  int count = 0;

    px_val.data=px_vec[count];
    py_val.data=py_vec[count];
    nx_val.data=nx_vec[count];
    ny_val.data=ny_vec[count];
    px_pub.publish(px_val);
    py_pub.publish(py_val);
    nx_pub.publish(nx_val);
    ny_pub.publish(ny_val);

  while (ros::ok())
  {
    if (count < px_vec.size()) {
    px_val.data=px_vec[count];
    py_val.data=py_vec[count];
    nx_val.data=nx_vec[count];
    ny_val.data=ny_vec[count];

    px_pub.publish(px_val);
    py_pub.publish(py_val);
    nx_pub.publish(nx_val);
    ny_pub.publish(ny_val);
    ROS_INFO("px %f", px_vec[count]);
		ROS_INFO("py %f", py_vec[count]);
		ROS_INFO("nx %f", nx_vec[count]);
		ROS_INFO("ny %f", ny_vec[count]);
    

    loop_rate.sleep();
    count=count+1;
    }
    ros::spinOnce();
  }


  return 0;
}
