// UMIGV
// Class to detect white lanes from an image
// Kushal Jaligama

#include "cv.h"

#include <cstdint>

#include <goat_msgs/PointArrayStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>

class LaneDetector {
 public:
  LaneDetector() = default;
  LaneDetector(ros::NodeHandle& nh);
  void transformImagePlaneToCameraPlane(const ros::TimerEvent &e);
  void grabZEDRawCB(const sensor_msgs::ImageConstPtr& msg);
  void grabDepthRegisteredCB(const sensor_msgs::ImageConstPtr& msg);
  ros::Timer makeTransformTimer(const ros::NodeHandle& nh, ros::Rate rate);

 private:
  ros::Publisher makePublisher(ros::NodeHandle& nh);
  image_transport::Subscriber makeDepthSubscriber();
  image_transport::Subscriber makeImageSubscriber();
  sensor_msgs::CameraInfoConstPtr getCameraInfo(ros::NodeHandle& nh);
  goat_msgs::PointArrayStamped makeMessage(
    std::vector<geometry_msgs::Point> points
  );
  sensor_msgs::ImageConstPtr makeMessage(cv::Mat input);
  image_transport::Publisher makeImagePublisher();

  cv::Mat rawImage;
  cv::Mat depthRegistered;

  // umigv::CannyFilter filter;
  umigv::MitchFilter filter;
  cv::Mat edges;

  ros::Publisher publisher;

  image_transport::ImageTransport it;
  image_transport::Subscriber depthSubscriber;
  image_transport::Subscriber imageSubscriber;
  image_transport::Publisher whitePublisher;

  image_geometry::PinholeCameraModel model;
  std::uint32_t whiteSequence = 0;
};
