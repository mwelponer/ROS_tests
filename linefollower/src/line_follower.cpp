#include <iostream>
#include <cstdlib>
#include <stdexcept>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


const bool VERBOSE_LOG = 0;
const bool GRAPHICAL_USER_INTERFACE = 0;
const float WINDOW_OUTPUT_PERCENTAGE = 50;

float NORMAL_SPEED = 0.3f;
const float FACTOR_LINEAR = 0.001f; // 0.001f
const float FACTOR_ANGULAR = 0.2f; // 0.2f

static const char* LEO_CAMERA_TOPIC = "/camera/image_raw";

struct RgbColor
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

  bool evenFrames = 1;

  struct RgbColor m_rgbToTrack;
  float m_colorErrorPercentage;  
  float speed;


public:
  /**
   * constructor
   */
  LineFollower(int argc, char **argv, RgbColor rgbToTrack, float colorErrorPercentage)
      : it(nh), m_rgbToTrack(rgbToTrack), m_colorErrorPercentage(colorErrorPercentage)
  {
    if(VERBOSE_LOG) std::cout << "LineFollower()" << std::endl;

    speed = NORMAL_SPEED;

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

    //cv::namedWindow("view");

    sub = it.subscribe(LEO_CAMERA_TOPIC, 1, &LineFollower::imageCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    int rate = 25;
    ros::Rate loop_rate(rate);
    if(VERBOSE_LOG) std::cout << "..LineFollower() DONE" << std::endl;    
  }

  /**
   * destructor
   */
  ~LineFollower()
  {
    cv::destroyWindow("crop");
    cv::destroyWindow("hsv");
    cv::destroyWindow("mask");
    cv::destroyWindow("masked_image");
    cv::destroyWindow("contours");
  }

  /**
   * callback called each time an image is received
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {

    if (msg == NULL) return;
    if (msg->encoding != "bgr8") return;
    if (evenFrames) { evenFrames = 0; return; } else { evenFrames = 1; } // skip even frames
    if(VERBOSE_LOG) std::cout << "  imageCallback()" << std::endl;

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

    //////////////////////////////////
    // resize into a smaller 20% image, and crop upper part to make it faster
    int height = cv_image.rows;
    int width = cv_image.cols;
    cv::Mat small_frame;
    cv::resize(cv_image, small_frame, cv::Size(int(width/100 * WINDOW_OUTPUT_PERCENTAGE), 
        int(height/100 * WINDOW_OUTPUT_PERCENTAGE)));
    if(GRAPHICAL_USER_INTERFACE)
    {
        //cv::imshow("small_frame", small_frame);
        //cv::waitKey(1);
    }


    //////////////////////////////////
    // crop out upper part
    cv::Mat crop_img;
    height = small_frame.rows;
    width = small_frame.cols;
    cv::Rect crop_region(0, int(height*2/5), width, int(height*3/5));
    crop_img = small_frame(crop_region);

    if(GRAPHICAL_USER_INTERFACE)
    {
      cv::imshow("crop", crop_img);
      //cv::waitKey(1);
    }


    
    //////////////////////////////////
    // Convert the image from RGB to HSV (more stable versus lighting conditions)
    cv::Mat hsv;
    cvtColor(crop_img, hsv, CV_BGR2HSV);
    
    if(GRAPHICAL_USER_INTERFACE)
    {
      cv::imshow("hsv", hsv);
      //cv::waitKey(1);
    }


    
    //////////////////////////////////
    // Convert rgb color to track to hsv and find the lower and upper values
    cv::Mat hsvToTrack = m_rgbToTrack.toHsv();
    //std::cout << "hsv: " << hsvToTrack << std::endl;
    cv::Vec3b hsvToTrack_vec3b = hsvToTrack.at<cv::Vec3b>(0,0);
    cv::Mat lower_color(1, 1, CV_8UC3);
    cv::Mat upper_color(1, 1, CV_8UC3);
    for(int i=0; i<3; i++)
    {
      lower_color.at<cv::Vec3b>(0,0)[i] = hsvToTrack_vec3b[i] * (1.0f - m_colorErrorPercentage / 100.0f);
      upper_color.at<cv::Vec3b>(0,0)[i] = hsvToTrack_vec3b[i] * (1.0f + m_colorErrorPercentage / 100.0f);
    }
    //std::cout << "hsv lower: " << lower_color.at<cv::Vec3b>(0,0) << std::endl;
    //std::cout << "hsv upper: " << upper_color.at<cv::Vec3b>(0,0) << std::endl;

    // Threshold the HSV image to get only the specific range of colors
    cv::Mat mask;//(height, width, CV_8UC3);
    cv::inRange(hsv, cv::Scalar(lower_color.at<cv::Vec3b>(0,0)[0],
                                lower_color.at<cv::Vec3b>(0,0)[1],
                                lower_color.at<cv::Vec3b>(0,0)[2]),
                     cv::Scalar(upper_color.at<cv::Vec3b>(0,0)[0],
                                upper_color.at<cv::Vec3b>(0,0)[1],
                                upper_color.at<cv::Vec3b>(0,0)[2]),
                                mask);

    if(GRAPHICAL_USER_INTERFACE)
    {
      cv::imshow("mask", mask);
      //cv::waitKey(1);
    }


    
    //////////////////////////////////
    // convert to 3 channels
    cv::cvtColor(mask, mask, CV_GRAY2BGR);
    //std::cout << "crop size: " << crop_img.cols << ", " << crop_img.rows << std::endl;
    //std::cout << "crop type: " << crop_img.type() << std::endl;
    //std::cout << "mask size: " << mask.cols << ", " << mask.rows << std::endl;
    //std::cout << "mask type: " << mask.type() << std::endl;

    //////////////////////////////////
    // Bitwise-AND mask and original image
    cv::Mat masked_image;
    cv::bitwise_and(crop_img, mask, masked_image);

    if(GRAPHICAL_USER_INTERFACE)
    {
      cv::imshow("masked_image", masked_image);
      //cv::waitKey(1);
    }


    
    //////////////////////////////////
    // Detect the contours
    //Prepare the image for findContours
    cv::cvtColor(mask, mask, CV_BGR2GRAY);
    cv::threshold(mask, mask, 128, 255, CV_THRESH_BINARY);
    //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
    std::vector<std::vector<cv::Point>> contours;
    cv::Point winner(-1, -1);
    cv::findContours(mask.clone(), contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_TC89_L1);
    if (VERBOSE_LOG) ROS_INFO_STREAM("  contours: " << contours.size());

    if (contours.size() > 0)
    {
      if (VERBOSE_LOG)
      {
        //Draw the contours
        cv::Mat contourImage(mask.size(), CV_8UC3, cv::Scalar(0,0,0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);
        for (size_t idx = 0; idx < contours.size(); idx++) {
            cv::drawContours(contourImage, contours, idx, colors[idx % 3]);
        }
        if(GRAPHICAL_USER_INTERFACE)
        {
          cv::imshow("contours", contourImage);
          //cv::waitKey(1);
        }
      }
    }
    else
    {
      // move the robot
      move_robot(height, width, winner.x, winner.y, speed, 0.3f);
      cv::waitKey(1); 

      return;
    }


    
    //////////////////////////////////
    // Find the centers of the contours
    std::vector<cv::Moments> mu(contours.size());
    double M00, M01, M10;
    std::vector<cv::Point> centers;
    for( int i = 0; i < contours.size(); i++ )
    {
      mu[i] = cv::moments(contours[i], false);
      M00 = mu[i].m00;
      M10 = mu[i].m10;
      M01 = mu[i].m01;
      try
      {
        centers.push_back(cv::Point(int(M10/M00), int(M01/M00)));
      }catch (std::runtime_error& e){
        ROS_WARN_STREAM("Exception Division by zero: " << e.what());
      }
      //std::cout << "(" << int(M10/M00) << ", " << int(M01/M00) << ")" << std::endl;
    }
    if (VERBOSE_LOG)
      for(int i=0; i<centers.size(); i++)
        ROS_INFO_STREAM("  center [" << centers[i].x << ", " << centers[i].y << "]");
    
    
    //////////////////////////////////
    // find the most centered & close centroid
    int index = 0; int candidate_index = 0;
    double min_x_dist_from_center = width/2;
    double max_y_value = 0;
    if(centers.size() > 0);
    {
      for(int i=0; i<centers.size(); i++)
      {
        cv::Point c = centers[i];
        double x_dist_from_center = abs(width/2 - c.x);

        // if the candidate has the same x as the actual max
        if (x_dist_from_center == min_x_dist_from_center)
        {
          if (c.y > max_y_value) // if the candidate is closer to us on the y
          {
            max_y_value = c.y;
            candidate_index = index;
          }
        }
        // if the candidate is at the moment the most right
        else if(x_dist_from_center < min_x_dist_from_center)
        {
            min_x_dist_from_center = x_dist_from_center;
            max_y_value = c.y;
            candidate_index = index;
        }
        index += 1;
      }

      if (VERBOSE_LOG) std::cout << "candidate centroid index: " << candidate_index << std::endl;

      winner = centers[candidate_index];
      if (VERBOSE_LOG)
        ROS_INFO_STREAM("    centroid [" << winner.x << ", " << winner.y << "]");
    }

    move_robot(height, width, winner.x, winner.y, speed, 0.3f);

    cv::waitKey(1); 
  }

  /**
   * move the robot
   */
  void move_robot(int height, int width, int cx, int cy, float linear_vel_base, float angular_vel_base)
  {
    //It move the Robot based on the Centroid Data
    //std::cout << "  move_robot()" << std::endl;
    twist_msg.linear.x = linear_vel_base;
    twist_msg.angular.z = angular_vel_base;

    if (cx > 0 && cy > 0)
    {
      // delta between the center of the frame and the centroid
      cv::Point delta(cx-width/2, cy-height/2);
      // -1 because when delta is positive we want to turn right, which means sending a negative angular
      twist_msg.angular.z = angular_vel_base * delta.x * FACTOR_ANGULAR * -1;
      //If its further away it has to go faster, closer then slower
      twist_msg.linear.x = linear_vel_base - delta.y * FACTOR_LINEAR;
    }
    else // if no centroid was detected
    {
      if (VERBOSE_LOG)
      {
        ROS_WARN_STREAM("NO CENTROID DETECTED");
        ROS_INFO_STREAM("SEARCHING...");
        //ROS_WARN_STREAM
        //ROS_ERROR_STREAM
        //ROS_FATAL_STREAM
      }

      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = angular_vel_base * 2.0;
    }

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

  ros::init(argc, argv, "line_follower");
  struct RgbColor rgbToTrack{45, 149, 62};
  LineFollower lf(argc, argv, rgbToTrack, 40.0f);
  lf.loop();
  cv::destroyWindow("view");

  return 0;
}
