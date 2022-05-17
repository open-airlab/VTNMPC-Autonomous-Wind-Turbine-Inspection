#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "airsim_ros_wrapper.h"
#include <ros/spinner.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImageConstPtr image;
  cv::Mat image_grayscale;
  try
  {

    image = cv_bridge::toCvShare(msg);
    image_grayscale = cv::Mat(image->image.size(), CV_8UC1);
    cv::convertScaleAbs(image->image, image_grayscale, 100, 0.0);


    //image_grayscale.convertTo(image_grayscale, CV_8U);
    //cv::normalize(image_grayscale, image_grayscale, 0, 255, cv::NORM_MINMAX);

    //std::cout << image_grayscale << std::endl;

    //cv::imshow("view", cv_bridge::toCvShare(msg, "32FC1")->image);
    cv::imshow("view_orig", image->image);

    cv::waitKey(30);
    
    //cv::Mat gray = cv::imread(cv_bridge::toCvShare(msg, "32FC1")->image);

    
   //ROS_INFO("cx: %f,cy: %f",avg_x,avg_y);
 



  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
  }

    //cv::normalize(image_grayscale, image_grayscale, 0, 1, cv::NORM_MINMAX);

    try
  {
    
    //cv::imshow("view", cv_bridge::toCvShare(msg, "32FC1")->image);
    cv::imshow("view_normal", image_grayscale);

    cv::waitKey(30);
    
    //cv::Mat gray = cv::imread(cv_bridge::toCvShare(msg, "32FC1")->image);

    
   //ROS_INFO("cx: %f,cy: %f",avg_x,avg_y);
 



  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
  }


   // image_grayscale.convertTo(image_grayscale, CV_32F, 1.0/255.0);
    
    
    
    
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::32FC1);

    std::vector<std::vector<cv::Point> > contours;
    
  
    cv::findContours(image_grayscale.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Fill holes in each contour
    cv::drawContours(image_grayscale, contours, -1, CV_RGB(255, 255, 255), -1);
   
    //cout << contours.size();
    double avg_x(0), avg_y(0); // average of contour points
    for (int j = 0; j < contours[1].size(); ++j)
    {
       avg_x += contours[1][j].x;
       avg_y += contours[1][j].y;
    }

    avg_x /= contours[1].size();
    avg_y /= contours[1].size();

    std::cout << avg_x << std::endl;

    try
  {
        //image_grayscale.convertTo(image_grayscale, CV_8U, 255.0);

    //cv::imshow("view", cv_bridge::toCvShare(msg, "32FC1")->image);
    cv::imshow("view", image_grayscale);

    cv::waitKey(30);
    
    //cv::Mat gray = cv::imread(cv_bridge::toCvShare(msg, "32FC1")->image);

    
   //ROS_INFO("cx: %f,cy: %f",avg_x,avg_y);
 



  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
  }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/airsim_node/PX4/inspection_camera/DepthVis", 1, imageCallback);
  //image_transport::Subscriber sub = it.subscribe("/airsim_node/PX4/inspection_camera/Scene", 1, imageCallback);





  ros::spin();
  cv::destroyWindow("view");
}



