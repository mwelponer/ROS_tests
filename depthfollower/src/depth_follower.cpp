#include <iostream>
#include <cstdlib>
#include <stdexcept>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <algorithm> 

#include <chrono>
#include <ctime>
#include <random>

const bool VERBOSE_LOG = 1;
const bool GRAPHICAL_USER_INTERFACE = 0;
const float WINDOW_OUTPUT_PERCENTAGE = 100;

float NORMAL_SPEED = 0.3f;
const float FACTOR_LINEAR = 0.2f; //0.003f;
const float FACTOR_ANGULAR = 0.2f; 

const int SCANNING_GRANULARITY = 31;
const int NUMBER_OF_SKIPPED_SIDE_BANDS = 2;
const int FILTER_WINDOW_SIZE = 8;

const float STEARING_DISTANCE = 0.4f;
const float GRAY_MAX_PIXEL_PERCENTAGE = 50; //???

const float IMPACT_DISTANCE = 0.02f;
const float BLACK_MAX_PIXEL_PERCENTAGE = 3.14f;

const int FRAMES_PER_SECOND = 7;
const int BACKWARD_TIME = 4;
const int SPINNING_TIME = 14;
const int STUCKING_TIME = 1;

//const int GRAY_MIN_PIXEL_COUNT = 50;

static const char* REALSENSE_CAMERA_COLOR_TOPIC = "/d435i_camera/color/image_raw";
static const char* REALSENSE_CAMERA_DEPTH_TOPIC = "/d435i_camera/depth/image_rect_raw";
bool evenFrames = 1;

struct Scan
{
  float slicesAvg[SCANNING_GRANULARITY];
  float filteredSlicesAvg[SCANNING_GRANULARITY];
  int grayPixelCount[SCANNING_GRANULARITY]; // used to stear
  int blackPixelCount[SCANNING_GRANULARITY]; // used to go backward
  int goodPixCountLeft;
  int goodPixCountRight;
  int closestIndexL;
  int farthestIndexL;
  int closestIndexR;
  int farthestIndexR;  

  void initialize()
  {
    //std::cout << "..initialize Scan" << std::endl;

    // initialize 
    for(int s = 0; s < SCANNING_GRANULARITY; s++)
    {
      slicesAvg[s] = 0.0f;
      filteredSlicesAvg[s] = 0.0f;
      grayPixelCount[s] = 0;
      blackPixelCount[s] = 0;
    }

    goodPixCountLeft = 0;
    goodPixCountRight = 0;
    closestIndexL = -1;
    closestIndexR = -1;
    farthestIndexL = -1;
    farthestIndexR = -1;
  }
};

struct Centroid
{
  int x, y;
};

class DepthFollower
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

  int indexes[SCANNING_GRANULARITY];
  float valuesSums[SCANNING_GRANULARITY];  
  float valuesBuffer[SCANNING_GRANULARITY][FILTER_WINDOW_SIZE];

  bool wall = false;
  bool rand_direction_is_set = false;
  bool goRight = false;

  std::chrono::time_point<std::chrono::steady_clock> t;
  int timeFrame;
  bool stuck;
  int count_backward;
  int count_spinning;
  int backwardFrames = FRAMES_PER_SECOND * BACKWARD_TIME;
  int spinningFrames = FRAMES_PER_SECOND * SPINNING_TIME;
  int stuckFrames = FRAMES_PER_SECOND * STUCKING_TIME;

  float depthMapAvgBuffer[FRAMES_PER_SECOND * STUCKING_TIME];

public:
  /**
   * constructor
   */
  DepthFollower(int argc, char **argv)
      : it(nh)
  {
    std::cout << "DepthFollower()" << std::endl;

    //t = std::chrono::steady_clock::now();
    timeFrame = 0;
    stuck = false;
    count_backward = 0;
    count_spinning = 0;


    // initialize global members
    for(int s = 0; s < SCANNING_GRANULARITY; s++)
    {
      indexes[s] = 0;
      valuesSums[s] = 0.0f;
      for(int f = 0; f < FILTER_WINDOW_SIZE; f++)
        valuesBuffer[s][f] = 0.0f;
    }

    for(int d = 0; d < stuckFrames; d++)
      depthMapAvgBuffer[d] = 0;
    
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
    //ros::init(argc, argv, "DepthFollower");

    twist_msg.linear.x=twist_msg.linear.y=twist_msg.linear.z=0;
    twist_msg.angular.x=twist_msg.angular.y=twist_msg.angular.z=0;

    sub = it.subscribe(REALSENSE_CAMERA_DEPTH_TOPIC, 1, &DepthFollower::imageCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    int rate = 25;
    ros::Rate loop_rate(rate);
    std::cout << "..DepthFollower() DONE" << std::endl;
  }

  /**
   * destructor
   */
  ~DepthFollower()
  {
    //cv::destroyWindow("view");
  }

  Scan scanFrame(const cv::Mat& frame)
  {
    Scan scan;
    scan.initialize();
    int pixCount[SCANNING_GRANULARITY];
    float valuesSums[SCANNING_GRANULARITY];

    int sliceWidth = (int) (frame.cols / SCANNING_GRANULARITY);

    for(int s = 0; s < SCANNING_GRANULARITY ; s++) // cycle on slices
    {      
      pixCount[s] = 0;
      valuesSums[s] = 0;

      for(int r = 0; r < frame.rows; r++) 
      {
         for(int c = sliceWidth * s; c < sliceWidth * (s + 1); c++)
         {
            float pixelVal = frame.at<uint16_t>(r, c) / 1000.0f;
            if(pixelVal != 0.0f && pixelVal < 7.0f) // skip black pixels and pixels that are too far
            {
              if(pixelVal <= IMPACT_DISTANCE)
                scan.blackPixelCount[s]++;
              if(pixelVal <= STEARING_DISTANCE) // keep track of close object 'volume'
                scan.grayPixelCount[s]++;
              valuesSums[s] += pixelVal;
              pixCount[s]++;

              // keep track of good left and right considered pixels
              if( s > NUMBER_OF_SKIPPED_SIDE_BANDS && s < (int)(SCANNING_GRANULARITY / 2) )
                scan.goodPixCountLeft++;
              else if( s > ((int)(SCANNING_GRANULARITY / 2)) && s < (SCANNING_GRANULARITY - NUMBER_OF_SKIPPED_SIDE_BANDS - 1) )
                scan.goodPixCountRight++;
            }
         }
      }
    }

    // evaluate the average for each scan slice
    for(int s = 0; s < SCANNING_GRANULARITY; s++)
    {	
      if (pixCount[s] > 0)
       	scan.slicesAvg[s] = valuesSums[s] / pixCount[s];
    }    
    
    return scan;
  }    

  cv::Mat resizeFramePercentage(const cv::Mat& frame, int percentage)
  {
    cv::Mat resized;
    cv::resize(frame, resized, 
        cv::Size(int(frame.cols / 100.0f * percentage), int(frame.rows / 100.0f * percentage)));

    return resized;
  }

  cv::Mat resizeFrameWidth(const cv::Mat& frame, int width)
  {
    float aspectRatio = float(frame.rows) / float(frame.cols);
    cv::Mat resized;
    cv::resize(frame, resized, 
        cv::Size(width, int(width * aspectRatio)));

    return resized;
  }


  /**
   * callback called each time an image is received
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    //std::cout << "imageCallback()" << std::endl;
    
    if (msg == NULL) return;
    if (evenFrames) { evenFrames = 0; return; } else { evenFrames = 1; } // skip even frames
    // timeFrame = (timeFrame+1) % 15;
    // if(timeFrame != 0) return;

    //if (VERBOSE_LOG)
    //{
      //auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t).count();
      //ROS_INFO_STREAM( elapsed << "ms elapsed since last imageCallback(), timeFrame: " << timeFrame);
      //t = std::chrono::steady_clock::now();
    //}

    cv::Mat depth_met; // 16bit metric depth
    cv::Mat depth_vis; // 8bit depth for visualization

    //std::cout << "encoding: " << msg->encoding << std::endl;
    
    if (msg->encoding == "16UC1")
    {
      cv_bridge::CvImagePtr cv_depth;
      try {
        cv_depth = cv_bridge::toCvCopy(msg);
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      depth_met = cv::Mat(cv_depth->image.rows, cv_depth->image.cols, CV_16UC1);
      depth_vis = cv::Mat(cv_depth->image.rows, cv_depth->image.cols, CV_8UC1);

      for(size_t i = 0; i < cv_depth->image.rows; i++)
      {
        unsigned short* Di = cv_depth->image.ptr<unsigned short>(i);
        unsigned short* Ii = depth_met.ptr<unsigned short>(i);

        char* Ivi = depth_vis.ptr<char>(i);
        for(size_t j = 0; j < cv_depth->image.cols; j++)
        {
          if(Di[j] > 0.0f)
          {
            Ii[j] = Di[j];
            Ivi[j] = (char) (255*((Di[j] / 1000.0)/(5.5))); // some suitable values.. => For visualization
          }
          else
          {
            Ii[j] = 0.0f;
            Ivi[j] = 0;
          }
        }
      }

      //cv::imshow("original", depth_vis);
      //cv::waitKey(30);
      //std::cout << "original: " << depth_vis.cols << "x" << depth_vis.rows << std::endl;
      

      //////////////////////////////////
      // resize into a smaller 20% image
      cv::Mat small_vis, small_met;      
      int height = depth_vis.rows;
      int width = depth_vis.cols;
      small_vis = resizeFramePercentage(depth_vis, WINDOW_OUTPUT_PERCENTAGE); //resizeFrameWidth(depth_vis, 155);
      small_met = resizeFramePercentage(depth_met, WINDOW_OUTPUT_PERCENTAGE); //resizeFrameWidth(depth_met, 155);
      int height_small = small_vis.rows;
      int width_small = small_vis.cols;

      std::string title = "";

      if(GRAPHICAL_USER_INTERFACE)
      {
        // visualize rectangle
        cv::rectangle(small_vis, cv::Point(0,0), cv::Point(width_small-1, int(height_small/2)-1), cv::Vec3b(255, 0, 0), 1);
        // visualize it 
        title.append(std::to_string(width_small)).append("x").append(std::to_string(height_small));
        cv::imshow(title, small_vis);
        cv::waitKey(30);
        //std::cout << "crop: " << small_vis.cols << "x" << small_vis.rows << std::endl;
      }

      //////////////////////////////////
      // crop out upper part
      cv::Mat crop_vis, crop_met;
      //cv::Rect crop_region(0, int(height*2/5), width, int(height*3/5));
      cv::Rect crop_region(0, 0, width_small, int(height_small/2));
      crop_vis = small_vis(crop_region);
      crop_met = small_met(crop_region);
      // update resolution values
      height = crop_vis.rows;
      width = crop_vis.cols;
      int sliceWidth = (int) (crop_vis.cols / SCANNING_GRANULARITY);

      if(GRAPHICAL_USER_INTERFACE)
      {
        // visualize slices
        //std::cout << "width: " << crop_vis.cols << ", SCANNING_GRANULARITY: " << SCANNING_GRANULARITY << std::endl;
        for(int s = 0; s <= SCANNING_GRANULARITY; s++)
        {
          int x = s * sliceWidth;
          //std::cout << "s: " << s << ", x: " << x << ", sliceWidth: " << sliceWidth << std::endl;
          cv::line(crop_vis, cv::Point(x, 0), cv::Point(x, height), cv::Scalar(150,150,150), 1, cv::LINE_4);
        }

        // visualize it 
        title = "";
        title.append(std::to_string(width)).append("x").append(std::to_string(height));
        cv::imshow(title, crop_vis);
        cv::waitKey(30);
        //std::cout << "crop: " << crop_vis.cols << "x" << crop_vis.rows << std::endl;
        //return;
      }

      ////////////////////////////////////
      // handle stuck rover case
      if(stuck)
      {
        //std::cout << "I AM IN STUCK !!!" << std::endl;
        Centroid winner;
        winner.y = (int)height/2;
        winner.x = (int)width/2;

        // drive backward
        if (count_backward < backwardFrames)
        {
          if(NORMAL_SPEED > 0) NORMAL_SPEED = -NORMAL_SPEED; // invert speed
          if (VERBOSE_LOG)
            ROS_INFO_STREAM("...BACKWARD");
        }

        if(count_backward >= backwardFrames && count_spinning <= spinningFrames) // stop backward 
        {
          // spinning
          if(NORMAL_SPEED < 0) NORMAL_SPEED = -NORMAL_SPEED; // restore speed
          winner.y = -1;
          winner.x = -1;

          if(count_spinning == spinningFrames) // exit stuck
          {
            std::cout << std::endl << "stuck FALSE!!! " << std::endl; 
            count_backward = -1;
            count_spinning = -1;

            for(int d = 0; d < stuckFrames; d++)
              depthMapAvgBuffer[d] = 0;

            winner.y = (int)height/2;
            winner.x = (int)width/2;        
            stuck = false;
          }

          count_spinning++;
        }
        
        count_backward++;

        std::cout << "count_backward: " << count_backward << ", count_spinning: " << count_spinning << std::endl;
        move_robot(height, width, winner.x, winner.y, NORMAL_SPEED, NORMAL_SPEED);
        return;
      }


      /////////////////////////////////////
      // Kalman filter Kv = Sv * (ratio) + Kv * (1-ratio)
      //averaged = ((averaged * (FILTER_WINDOW_SIZE -1)) + value) / FILTER_WINDOW_SIZE;
      //std::cout << "avg distance: " << value << " ( " << averaged  << ")" <<  std::endl;


      /////////////////////////////////////
      // Moving Average Filter
      Scan scan = scanFrame(crop_met);

      std::string scanString = "";
      float closestValueL = 1000.0f;
      float farthestValueL = 0.0f;
      float closestValueR = 1000.0f;
      float farthestValueR = 0.0f;      

      //float filteredSlicesAvgSum = 0;
      int blackPixTotalCount = 0;
      int leftGrayPixTotalCount = 0;
      int rightGrayPixTotalCount = 0;

      for(int s = 0; s < SCANNING_GRANULARITY; s++){
        valuesSums[s] -= valuesBuffer[s][indexes[s]]; // remove the oldest value entry
        valuesBuffer[s][indexes[s]] = scan.slicesAvg[s]; // add the newest value
        valuesSums[s] += scan.slicesAvg[s]; // add the newest value to the sum
        indexes[s] = (indexes[s]+1) % FILTER_WINDOW_SIZE; // update index
        scan.filteredSlicesAvg[s] = valuesSums[s] / FILTER_WINDOW_SIZE; // evaluate the filtered value
        
        // keep track of the total number of black pixels
        blackPixTotalCount += scan.blackPixelCount[s];

        // keep track of the left/right number of gray pixels
        if( s > NUMBER_OF_SKIPPED_SIDE_BANDS && s < (int)(SCANNING_GRANULARITY / 2) )
        { 
          // left part
          leftGrayPixTotalCount += scan.grayPixelCount[s]; 
          if(scan.filteredSlicesAvg[s] > farthestValueL){ scan.farthestIndexL = s; farthestValueL = scan.filteredSlicesAvg[s];}
          if(scan.filteredSlicesAvg[s] < closestValueL){ scan.closestIndexL = s; closestValueL = scan.filteredSlicesAvg[s];}
        }
        else if( s > ((int)(SCANNING_GRANULARITY / 2)) && s < (SCANNING_GRANULARITY - NUMBER_OF_SKIPPED_SIDE_BANDS - 1) )
        {
          rightGrayPixTotalCount += scan.grayPixelCount[s];
          if(scan.filteredSlicesAvg[s] > farthestValueR){ scan.farthestIndexR = s; farthestValueR = scan.filteredSlicesAvg[s];}
          if(scan.filteredSlicesAvg[s] < closestValueR){ scan.closestIndexR = s; closestValueR = scan.filteredSlicesAvg[s];}          
        }

        // update closes and farthest indices
        //if(scan.filteredSlicesAvg[s] > farthestValue){ scan.farthestIndex = s; farthestValue = scan.filteredSlicesAvg[s];}
        //if(scan.filteredSlicesAvg[s] < closestValue){ scan.closestIndex = s; closestValue = scan.filteredSlicesAvg[s];}
        
        scanString.append(std::to_string(scan.filteredSlicesAvg[s]));
        if(s < (SCANNING_GRANULARITY - 1)) scanString.append(" | ");
      }
      //ROS_INFO_STREAM(scanString);


      //////////////////////////////////////
      // check if the rover is STUCK !!
      // evaluate the percentage of black frame pixels
      float blackPixFramePercentage = blackPixTotalCount*100.0/(scan.goodPixCountLeft + scan.goodPixCountRight);
      // evaluate the percentage of gray left and right pixels
      // std::cout << "leftGrayPixTotalCount: " << leftGrayPixTotalCount << ", scan.goodPixCountLeft: " << scan.goodPixCountLeft << std::endl;
      // std::cout << "rightGrayPixTotalCount: " << rightGrayPixTotalCount << ", scan.goodPixCountRight: " << scan.goodPixCountRight << std::endl;

      float leftGrayPercentage = leftGrayPixTotalCount*100.0/scan.goodPixCountLeft;
      float rightGrayPercentage = rightGrayPixTotalCount*100.0/scan.goodPixCountRight;

      // std::cout << "BLACK: " << blackPixFramePercentage 
      //     << ", GRAY LEFT: " << leftGrayPercentage
      //     << ", GRAY RIGHT: " << rightGrayPercentage << std::endl;

      // check if we are stuck
      if(blackPixFramePercentage > BLACK_MAX_PIXEL_PERCENTAGE)
      {
        ROS_WARN_STREAM("THE ROVER IS STUCK !!! (BLACK % " << blackPixFramePercentage << ")" );
        stuck = true;
        return;
      }
      

      if(GRAPHICAL_USER_INTERFACE)
      {
        ////////////////////////////////      
        // visualize farthest & closest slice rectangle
        cv::rectangle(crop_vis, cv::Point(scan.farthestIndexL*sliceWidth, 0), 
            cv::Point(scan.farthestIndexL*sliceWidth+sliceWidth, height), cv::Vec3b(255, 255, 255), -1);
        cv::rectangle(crop_vis, cv::Point(scan.farthestIndexR*sliceWidth, 0), 
            cv::Point(scan.farthestIndexR*sliceWidth+sliceWidth, height), cv::Vec3b(255, 255, 255), -1);          
        //cv::rectangle(crop_vis, cv::Point(scan.closestIndex*sliceWidth, 0), 
        //    cv::Point(scan.closestIndex*sliceWidth+sliceWidth, height), cv::Vec3b(10, 10, 10), -1);
        cv::imshow(title, crop_vis);
        cv::waitKey(30);
      }


      /////////////////////////////////
      // centroid simulation
      float linear = NORMAL_SPEED;
      float angular = NORMAL_SPEED;      

      Centroid winner;
      winner.y = (int)height/2;

      // int stearingValue = 0;
      // if(scan.farthestIndex < SCANNING_GRANULARITY / 2)
      //   stearingValue = std::abs(scan.farthestIndex - SCANNING_GRANULARITY / 2);
      // else
      //   stearingValue = scan.farthestIndex - SCANNING_GRANULARITY / 2; 
      
      //int sliceWidth = (int)(width / SCANNING_GRANULARITY);
      int centroid_x_L = scan.farthestIndexL * sliceWidth + (int)(sliceWidth / 2);
      int centroid_x_R = scan.farthestIndexR * sliceWidth + (int)(sliceWidth / 2);
      // int leftIndex = 7; //(int) (SCANNING_GRANULARITY / 4);
      // int rightIndex = 23; //(int) (SCANNING_GRANULARITY / 2) + leftIndex;


      ///////////////////////////////////
      // guidance directions
      if(wall)
      {
        if (VERBOSE_LOG)
        {    
          ROS_WARN_STREAM(std::setprecision(2) << "WALL         L_gray: " << leftGrayPercentage << "% (" 
                  << leftGrayPixTotalCount << ")\tR_gray: " << rightGrayPercentage << "% (" << rightGrayPixTotalCount << ")");
        }
          
        winner.x = -1;
        winner.y = -1;
        //std::cout << "THE ROVER IS STUCK !!!" << std::endl;
        //stuck = true;
      }
    
      if( leftGrayPercentage < GRAY_MAX_PIXEL_PERCENTAGE && rightGrayPercentage < GRAY_MAX_PIXEL_PERCENTAGE)
      {
        if (VERBOSE_LOG)
        {
          ROS_INFO_STREAM(std::setprecision(2) << "FORWARD      L_gray: " << leftGrayPercentage << "% (" 
                  << leftGrayPixTotalCount << ")\tR_gray: " << rightGrayPercentage << "% (" << rightGrayPixTotalCount << ")");
        }

        linear = 1;
        winner.x = (int)width/2;
        wall = false;
        rand_direction_is_set = false;
      }      
      else if( !wall && leftGrayPercentage < GRAY_MAX_PIXEL_PERCENTAGE && rightGrayPercentage > GRAY_MAX_PIXEL_PERCENTAGE)
      {
        if (VERBOSE_LOG)
        {    
          ROS_INFO_STREAM(std::setprecision(2) << "LEFT >>>>>>> L_gray: " << leftGrayPercentage << "% (" 
                  << leftGrayPixTotalCount << ")\tR_gray: " << rightGrayPercentage << "% (" << rightGrayPixTotalCount << ")");
        }
       
        winner.x = centroid_x_L; //(int)width*2/9;
      }
      else if( !wall && leftGrayPercentage > GRAY_MAX_PIXEL_PERCENTAGE 
          && rightGrayPercentage < GRAY_MAX_PIXEL_PERCENTAGE)
      {
        if (VERBOSE_LOG)
        {    
          ROS_INFO_STREAM(std::setprecision(2) << "RIGHT <<<<<< L_gray: " << leftGrayPercentage << "% (" 
                  << leftGrayPixTotalCount << ")\tR_gray: " << rightGrayPercentage << "% (" << rightGrayPixTotalCount << ")");
        }
        
        winner.x = centroid_x_R; //(int)width*7/9;
      }
      else
      {
        if(scan.filteredSlicesAvg[15] <= IMPACT_DISTANCE) 
          wall = true; 
        
        else 
        {
          if(!rand_direction_is_set){
            goRight = rand() % 2 == 1;
            rand_direction_is_set = true; 
          }
          
          //if(leftGrayPercentage > rightGrayPercentage)
          if(goRight)
            winner.x = centroid_x_R;
          else
            winner.x = centroid_x_L; 

          ROS_WARN_STREAM("cazzoculo figa random is: " << goRight);
          move_robot(height, width, winner.x, winner.y, linear, NORMAL_SPEED);
          
          return;
        }
        

        if (VERBOSE_LOG)
        {
          ROS_WARN_STREAM(std::setprecision(2) << "STOP         L_gray: " << leftGrayPercentage << "% (" 
                  << leftGrayPixTotalCount << ")\tR_gray: " << rightGrayPercentage << "% (" << rightGrayPixTotalCount << ")");
        }

        winner.x = -1;
        winner.y = -1;
      }  

      // move the robot
      move_robot(height, width, winner.x, winner.y, linear, NORMAL_SPEED);
      //cv::waitKey(30);
    }
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
        if(stuck)
          ROS_INFO_STREAM("...SPINNING "); //(GRAY LEFT % " << leftGrayPercentage << "GRAY RIGHT % " << rightGrayPercentage << ")");
        else
          ROS_INFO_STREAM("SEARCHING..."); //(GRAY LEFT % " << leftGrayPercentage << "GRAY RIGHT % " << rightGrayPercentage << ")");
        //ROS_WARN_STREAM
        //ROS_ERROR_STREAM
        //ROS_FATAL_STREAM
      }

      twist_msg.linear.x = 0.0;
      //auto gen = std::bind(std::uniform_int_distribution<>(0,1), std::default_random_engine());
      //bool b = gen();

      if(goRight)
        twist_msg.angular.z = angular_vel_base * 2.0;
      else
        twist_msg.angular.z = angular_vel_base * -2.0;
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

  ros::init(argc, argv, "depth_follower_start");
  DepthFollower df(argc, argv);
  df.loop();

  return 0;
}
