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
  

    image = cv_bridge::toCvShare(msg);
    image_grayscale = cv::Mat(image->image.size(), CV_8UC1);
    cv::convertScaleAbs(image->image, image_grayscale, 100, 0.0);

  


    cv::threshold(image_grayscale, image_grayscale, 100, 255, cv::THRESH_BINARY);


    signed int height=image_grayscale.rows;
    signed int width=image_grayscale.cols;
    signed int pixelValue;
    signed int cx;
    signed int cy;
    signed int n;
    n=0;
    cx=0;
    cy=0;
    for (int i=1;i<height;++i){
    for (int j=1; j<width; ++j){

    pixelValue = (int)image_grayscale.at<uchar>(i,j);
    if (pixelValue >50){
        cx=cx+j;
        cy=cy+i;
        n=n+1;
    }
    }
    }
   cx=cx/n;

  std::cout<< cx << std::endl;
  std::cout<< width << std::endl;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/airsim_node/PX4/inspection_camera/DepthVis", 1, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
}



