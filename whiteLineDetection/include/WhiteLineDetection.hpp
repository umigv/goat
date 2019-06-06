#ifndef WHITELINE_HPP
#define WHITELINE_HPP


#include <iostream>
#include <cstdlib>

#include <sl/Core.hpp>
#include <sl/Camera.hpp>

#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <numeric>
#include <random>
#include <system_error>
#include <type_traits>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <simt_tf/simt_tf.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

class DetectWhiteLines{

public:
  DetectWhiteLines(const ros::NodeHandle &n)
  : node(n), occ_pub(node.advertise<sensor_msgs::PointCloud2>("white_lines", 10))
  {
		xzMat = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0,0,0));
		outputImage = cv::Mat(WIDTH, HEIGHT, CV_8UC4, cv::Scalar(0));
		seqId = 0;
		initCamera();
  }
  
  DetectWhiteLines(const DetectWhiteLines & other);

  ~DetectWhiteLines(){zed.close();}

  void imuTransform(const sensor_msgs::ImuConstPtr &imu);
  void detect(const ros::TimerEvent&);
  void loadBuffer(tf2_ros::Buffer * b) { buffer = b;}


private:
  const int MAX_Y_VALUE = 8;
  const int MIN_Y_VALUE = -5;
  const int MAX_X_VALUE = 7;
  const int MIN_X_VALUE = 0;
  const double YDIVISOR = 0.01;
  const double XDIVISOR = 0.01;
  const int HEIGHT = 1280;
  const int WIDTH = 720;
  const int SHIFTVAL = 6;

  sl::InitParameters initParameters;

  sl::Camera zed;
  ros::NodeHandle node;
  sl::Mat point_cloud;
  cv::Mat xzMat;
  cv::Mat outputImage;
  tf2::Quaternion imuQuat;
  ros::Publisher occ_pub;
  geometry_msgs::TransformStamped bodyFrame;
  tf2_ros::Buffer * buffer;
  int seqId;
  bool valid = true;
  
  
  bool initCamera();
  bool loadPointCloud();
  void convertXZ();
  void run();
  void transformIMU(int & x, int & z);
  void whiteLineDetection();
  bool isValidPoint(float currVal, bool isX);
  void publish();
  
  void convertXZGPU();

  
  struct Rgba
  {
		std::uint8_t r;
		std::uint8_t g;
		std::uint8_t b;
		std::uint8_t a;
  };

  Rgba unpack_float(float x)
  {
		union FloatPair 
		{
			float f;
			Rgba rgba;
		};

		FloatPair pair{ x };

		return pair.rgba;
  }
    
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

  void clearXZ() { xzMat = cv::Mat(WIDTH, HEIGHT, CV_8UC4, cv::Scalar(0,0,0));}
};

#endif
