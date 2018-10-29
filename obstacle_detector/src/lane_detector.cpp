// UMIGV
// Implementation of Lane Detection - Can run standalone on OpenCV images
// Now adapted for Point Clouds from ZED Stereocamera
// Kushal Jaligama

#include "lane_detector.h"

#include <cmath>
#include <functional>
#include <iostream>
#include <mutex>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/image_encodings.h>

namespace detail {

static inline geometry_msgs::Point point_from_cv(
    const cv::Point3d& cv) noexcept {
  geometry_msgs::Point point;

  point.x = cv.x;
  point.y = cv.y;
  point.z = cv.z;

  return point;
}

}  // namespace detail

LaneDetector::LaneDetector(ros::NodeHandle& nh)
    : publisher(makePublisher(nh)),
      it(nh),
      depthSubscriber(makeDepthSubscriber()),
      imageSubscriber(makeImageSubscriber()),
      whitePublisher(makeImagePublisher()) {
  ROS_DEBUG_STREAM("Getting ZED camera info in constructor");

  const auto info = getCameraInfo(nh);  // Wait for ZED to send the camera info
  model.fromCameraInfo(info);
}

void LaneDetector::transformImagePlaneToCameraPlane(const ros::TimerEvent&) {
  if (rawImage.empty() || depthRegistered.empty()) {
    return;
  }

  edges = filter.filter(rawImage);  // whiteCannyEdges
  const auto canny_msg = makeMessage(edges);
  whitePublisher.publish(canny_msg);

  std::vector<geometry_msgs::Point> points;
  points.reserve(edges.total());

  for (int row = 0; row < edges.rows; ++row) {
    for (int col = 0; col < edges.cols; ++col) {
      const double depth = depthRegistered.at<float>(row, col);

      if (!edges.at<uchar>(row, col) || !std::isfinite(depth)) {
        return;
      }

      const auto u = static_cast<double>(col);
      const auto v = static_cast<double>(row);

      const cv::Point2d uv_rect{u, v};

      const cv::Point3d ray = model.projectPixelTo3dRay(uv_rect);
      const double ray_norm = std::sqrt(ray.dot(ray));

      const double depth_f64 = depth;

      const cv::Point3d projected = ray * (depth_f64 / ray_norm);
      const auto to_add = detail::point_from_cv(projected);
      points.push_back(to_add);
    }
  }

  const auto message = makeMessage(std::move(points));

  publisher.publish(message);
}

void LaneDetector::grabZEDRawCB(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  rawImage = cv_ptr->image;
}

void LaneDetector::grabDepthRegisteredCB(
    const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  depthRegistered = cv_ptr->image;
}

ros::Timer LaneDetector::makeTransformTimer(const ros::NodeHandle& nh,
                                            const ros::Rate rate) {
  return nh.createTimer(rate, &LaneDetector::transformImagePlaneToCameraPlane,
                        this);
}

ros::Publisher LaneDetector::makePublisher(ros::NodeHandle& nh) {
  static constexpr char TOPIC[] = "camera/white_line_points";
  static constexpr std::uint32_t QUEUE_SIZE = 10;

  return nh.advertise<goat_msgs::PointArrayStamped>(TOPIC, QUEUE_SIZE);
}

image_transport::Subscriber LaneDetector::makeDepthSubscriber() {
  static constexpr char TOPIC[] = "camera/left/image_rect";
  static constexpr std::uint32_t QUEUE_SIZE = 10;
  static constexpr auto CALLBACK_PTR = &LaneDetector::grabZEDRawCB;

  return it.subscribe(TOPIC, QUEUE_SIZE, CALLBACK_PTR, this);
}

image_transport::Subscriber LaneDetector::makeImageSubscriber() {
  static constexpr char TOPIC[] = "camera/depth/image";
  static constexpr std::uint32_t QUEUE_SIZE = 10;
  static constexpr auto CALLBACK_PTR = &LaneDetector::grabDepthRegisteredCB;

  return it.subscribe(TOPIC, QUEUE_SIZE, CALLBACK_PTR, this);
}

image_transport::Publisher LaneDetector::makeImagePublisher() {
  static constexpr char TOPIC[] = "camera/white_filter_image";
  static constexpr std::uint32_t QUEUE_SIZE = 10;

  return it.advertise(TOPIC, QUEUE_SIZE);
}

sensor_msgs::CameraInfoConstPtr LaneDetector::getCameraInfo(
    ros::NodeHandle& nh) {
  using MessageT = sensor_msgs::CameraInfo;

  static constexpr char TOPIC[] = "camera/left/camera_info";
  static const ros::Duration TIMEOUT{10};

  return ros::topic::waitForMessage<MessageT>(TOPIC, nh, TIMEOUT);
}

// pass by value to forward both const& and && with minimal cost
goat_msgs::PointArrayStamped LaneDetector::makeMessage(
    std::vector<geometry_msgs::Point> points) {
  static constexpr char FRAME_ID[] = "zed_depth_camera";

  goat_msgs::PointArrayStamped message;

  message.header.frame_id = FRAME_ID;
  message.header.seq = whiteSequence++;

  message.points = std::move(points);

  return message;
}

// pass by value to forward both const& and && with minimal cost
sensor_msgs::ImageConstPtr LaneDetector::makeMessage(cv::Mat input) {
  static constexpr char FRAME_ID[] = "zed_left_camera";

  sensor_msgs::ImageConstPtr message =
      cv_bridge::CvImage(std_msgs::Header(),
                         sensor_msgs::image_encodings::TYPE_8UC1, input)
          .toImageMsg();

  return message;
}
