#include <iostream>
#include <cstdlib>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

typedef struct RgbColor
{
  int r,g,b;

  cv::Mat /*cv::Vec3b*/ toHsv()
  {
    cv::Mat hsv;
    cv::Mat bgr(1, 1, CV_8UC3);
    bgr.at<cv::Vec3b>(0, 0) = cv::Vec3b(b, g, r);
    cvtColor(bgr, hsv, CV_BGR2HSV);

    //return hsv.at<cv::Vec3b>(0,0);
    return hsv;
  }
};

class LineFollower
{
private:
  struct RgbColor m_rgbToTrack;
  float m_colorErrorPercentage;
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;

  geometry_msgs::Twist twist_msg;
  ros::Publisher cmd_vel_pub;


public:
  /**
   * constructor
   */
  LineFollower(int argc, char **argv, RgbColor rgbToTrack, float colorErrorPercentage)
      : it(nh), m_rgbToTrack(rgbToTrack), m_colorErrorPercentage(colorErrorPercentage)
  {
    std::cout << "LineFollower()" << std::endl;

    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    //ros::init(argc, argv, "lineFollower");

    twist_msg.linear.x=twist_msg.linear.y=twist_msg.linear.z=0;
    twist_msg.angular.x=twist_msg.angular.y=twist_msg.angular.z=0;

    cv::namedWindow("view");

    sub = it.subscribe("/camera/image_raw", 1, &LineFollower::imageCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    int rate=50;
    ros::Rate loop_rate(rate);
  }

  /**
   * destructor
   */
  ~LineFollower()
  {
    cv::destroyWindow("view");
  }

  /**
   * callback called each time an image is received
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    //std::cout << "  imageCallback()" << std::endl;

    cv::Mat cv_image;

    try
    {
      //cv::imshow( "view", cv_bridge::toCvShare(msg, "bgr8")->image );
      cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      return;
    }

    //cv::imshow("view", cv_image);
    //cv::waitKey(30);

    // resize into a smaller 20% image, and crop upper part to make it faster
    int height = cv_image.rows;
    int width = cv_image.cols;
    cv::Mat small_frame;
    cv::resize(cv_image, small_frame, cv::Size(int(width/100*20), int(height/100*20)));
    //cv::imshow("small_frame", small_frame);
    //cv::waitKey(30);

    // crop out upper part
    cv::Mat crop_img;
    height = small_frame.rows;
    width = small_frame.cols;
    cv::Rect crop_region(0, int(height*2/5), width, int(height*3/5));
    crop_img = small_frame(crop_region);
    //cv::imshow("crop", crop_img);
    //cv::waitKey(30);

    // Convert the image from RGB to HSV (more stable versus lighting conditions)
    cv::Mat hsv;
    cvtColor(crop_img, hsv, CV_BGR2HSV);
    //cv::imshow("hsv", hsv);
    //cv::waitKey(30);

    // Convert rgb color to track to hsv and find the lower and upper values
    cv::Mat hsvToTrack = m_rgbToTrack.toHsv();
    //std::cout << "hsv: " << hsvToTrack << std::endl;
    cv::Vec3b hsvToTrack_vec3b = hsvToTrack.at<cv::Vec3b>(0,0);
    cv::Mat lower_color = hsvToTrack;
    cv::Mat upper_color = hsvToTrack;
    for(int i=0; i<3; i++)
    {
      lower_color.at<cv::Vec3b>(0,0)[i] = hsvToTrack_vec3b[i] * (1.0f - m_colorErrorPercentage / 100.0f);
      upper_color.at<cv::Vec3b>(0,0)[i] = hsvToTrack_vec3b[i] * (1.0f + m_colorErrorPercentage / 100.0f);
    }
    std::cout << "hsv lower: " << lower_color.at<cv::Vec3b>(0,0) << std::endl;
    std::cout << "hsv upper: " << upper_color.at<cv::Vec3b>(0,0) << std::endl;

    // Threshold the HSV image to get only the specific range of colors
    cv::Mat mask;
    cv::inRange(hsv, lower_color, upper_color, mask);
    cv::imshow("mask", mask);
    //cv::waitKey(30);

    // Bitwise-AND mask and original image
    cv::Mat masked_image;
    cv::bitwise_and(crop_img, mask, masked_image);
    cv::imshow("masked_image", masked_image);
    cv::waitKey(30);

    // detect the contours

    // find the most centered&close centroid

    // move the robot



    // test move
    // move_robot(0.1f, 0.0);
  }

  /**
   * move the robot
   */
  void move_robot(float linear_vel_base, float angular_vel_base)
  {
    std::cout << "  move_robot()" << std::endl;
    twist_msg.linear.x = linear_vel_base;
    twist_msg.angular.z = angular_vel_base;
    cmd_vel_pub.publish(twist_msg);
  }

  /**
   * ros spin
   */
  void loop()
  {
    ros::spin();
  }

};


/**
 * main function
 */
int main(int argc, char **argv)
{
  std::cout << "main()" << std::endl;

  ros::init(argc, argv, "line_follower_start");
  struct RgbColor rgbToTrack{45, 149, 62};
  LineFollower lf(argc, argv, rgbToTrack, 40.0f);
  lf.loop();

  return 0;
}
