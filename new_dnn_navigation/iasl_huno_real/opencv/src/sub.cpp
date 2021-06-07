#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <std_msgs/UInt8MultiArray.h>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}


// void imageCallback(const std_msgs::UInt8MultiArray::ConstPtr& array)
// {
//   try
//   {
//     cv::Mat frame = cv::imdecode(array->data, 1);
//     cv::imshow("view", frame);
//     cv::waitKey(1);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("cannot decode image");
//   }
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "opencv_sub");

//   cv::namedWindow("view");
//   cv::startWindowThread();

//   ros::NodeHandle nh;
//   ros::Subscriber sub = nh.subscribe("camera/image", 5, imageCallback);

//   ros::spin();
//   cv::destroyWindow("view");
// }
