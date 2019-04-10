#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <string>

#define gps_topic_name "/move_base/goal"
#define costmap_topic_name "/move_base/local_costmap"

// Costmap
// Header
struct Header {
  int seq;
  int stamp_sec;
  int stamp_nsec;
  string frame_id;
};
Header costmapHeader;

//Map meta data
int load_time_sec;
int load_time_nsec;
float resolution;
int width;
int height;

struct Quaternion {
  float x;
  float y;
  float z;
  float w;
};

struct Point {
  float x;
  float y;
  float z;
};

struct Vector3 {
  float x;
  float y;
  float z;
}

struct Twist {
  Vector3 linear;
  Vector3 angular;
}

struct Pose {
  Point position;
  Quaternion orientation;
};
Pose costMapPose;

int* data = new int[1];

// GPS
Header gpsHeader;
string child_frame_id;
Pose gpsPose;
float gpsPoseCovariance[36];
Twist gpsTwist;
float gpsTwistCovariance[36];

void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Fill out header
  this->gpsmapHeader.seq = msg->header.seq;
  this->gpsmapHeader.stamp_sec = msg->header.stamp.sec;
  this->gpsmapHeader.stamp_nsec = msg->header.stamp.nsec;
  this->gpsmapHeader.frame_id = msg->header.frame_id;

  this->child_frame_id = msg->child_frame_id;

  this->gpsPose.position.x = msg->pose.pose.position.x;
  this->gpsPose.position.y = msg->pose.pose.position.y;
  this->gpsPose.position.z = msg->pose.pose.position.z;

  this->gpsPose.orientation.x = msg->pose.orientation.x;
  this->gpsPose.orientation.y = msg->pose.orientation.y;
  this->gpsPose.orientation.z = msg->pose.orientation.z;
  this->gpsPose.orientation.w = msg->pose.orientation.w;

  for(int i = 0; i < 36; i++) {
    this->gpsPoseCovariance[i] = msg->pose.covariance[i];
    this->gpsTwistCovariance[i] = msg->twist.covariance[i];
  }

  this->gpsTwist.linear.x = msg->twist.linear.x;
  this->gpsTwist.linear.y = msg->twist.linear.y;
  this->gpsTwist.linear.z = msg->twist.linear.z;

  this->gpsTwist.angular.x = msg->twist.angular.x;
  this->gpsTwist.angular.y = msg->twist.angular.y;
  this->gpsTwist.angular.z = msg->twist.angular.z;

}

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  // Fill out header
  this->costmapHeader.seq = msg->header.seq;
  this->costmapHeader.stamp_sec = msg->header.stamp.sec;
  this->costmapHeader.stamp_nsec = msg->header.stamp.nsec;
  this->costmapHeader.frame_id = msg->header.frame_id;

  // Fill out meta data
  this->load_time_sec = msg->info.map_load_time.sec;
  this->load_time_nsec = msg->info.map_load_time.nsec;
  this->resolution = msg->info.resolution;
  this->width = msg->info.width;
  this->height = msg->info.height;

  this->costMapPose.orientation.x = msg->info.origin.orientation.x;
  this->costMapPose.orientation.y = msg->info.origin.orientation.y;
  this->costMapPose.orientation.z = msg->info.origin.orientation.z;
  this->costMapPose.orientation.w = msg->info.origin.orientation.w;

  this->costMapPose.position.x = msg->info.origin.position.x;
  this->costMapPose.position.y = msg->info.origin.position.y;
  this->costMapPose.position.z = msg->info.origin.position.z;

  delete[] this->data;
  this->data = new int[this->width * this->height];
  for(int i = 0; i < (this->width * this->height); i++) {
    this->data[i] = msg->data[i];
  }

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber gps_sub = n.subscribe(gps_topic_name, 1000, gpsCallback);
  ros::Subscriber costmap_sub = n.subscribe(costmap_topic_name, 1000, costmapCallback);

  ros::spin();

  return 0;
}