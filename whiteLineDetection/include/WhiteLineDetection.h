#include <iostream>
#include <cstdlib>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>



using namespace std;
using namespace sl;

class DetectWhiteLines{

public:
 DetectWhiteLines(const ros::NodeHandle n) : it{n}
  {
    node = n;
    xzMat = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0,0,0));
    outputImage = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0,0,0));
    whiteLineDetect =  cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0,0,0));
    pub = it.advertise("WhiteLineDetection",1);
  }

  ~DetectWhiteLines(){zed.close();}

  bool initCamera();
  bool loadPointCloud();
  double findMinX();
  void convertXZ();
  void run();
  
  void whiteLineDetection();

  DetectWhiteLines(const DetectWhiteLines & other);
  
  void imuTransform(const sensor_msgs::ImuConstPtr &imu);
  void detect(const ros::TimerEvent&);
  
  void displayXZ(int time)
  {
     cv::imshow("XZ_transform",xzMat);
     cv::waitKey(time);
  }
  void displayWL(int time)
  {
     cv::imshow("WhiteLine",outputImage);
     cv::waitKey(time);
  }

  void clearXZ()
  {
     xzMat = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0,0,0));
     cout << "CLEARED\n";
  }

  void transformIMU(int & x, int & z);
  
private:
  const int MAX_X_VALUE = 12;
  const int MIN_X_VALUE = -2;
  const int MAX_Z_VALUE = 7;
  const int MIN_Z_VALUE = 0;
  const double XDIVISOR = 0.01;
  const double ZDIVISOR = 0.01;
  const int HEIGHT = 1280;
  const int WIDTH = 720;
  const int SHIFTVAL = 4;

  InitParameters initParameters;

  Camera zed;
  sl::Mat point_cloud;
  cv::Mat xzMat;
  cv::Mat whiteLineDetect;
  cv::Mat outputImage;
  image_transport::Publisher pub;
  image_transport::ImageTransport it;
  sensor_msgs::Imu imu_val;
  tf2::Quaternion quat;
  
  ros::NodeHandle node;
  
  struct Rgba {
   std::uint8_t r;
   std::uint8_t g;
   std::uint8_t b;
   std::uint8_t a;
  };

  Rgba unpack_float(float x) {
    union FloatPair {
      float f;
      Rgba rgba;
    };

    FloatPair pair{ x };

    return pair.rgba;
  }

  bool isValidPoint(float  currVal, bool isX);
};
