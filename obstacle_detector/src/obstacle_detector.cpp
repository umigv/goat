// UMIGV
// Get data from a ZED Camera and do OpenCV processing on it
// Kushal Jaligama

#include "lane_detector.h"

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

int main(int argc, char* argv[]) {
  // Start ROS
  ros::init(argc, argv, "obstacle_detector_node");
  ros::NodeHandle nh;

  LaneDetector ld(nh);

  const auto timer = ld.makeTransformTimer(nh, ros::Rate{ 10.0 });

  ROS_INFO_STREAM("Starting loop");
  ros::spin();

  return 0;
}
