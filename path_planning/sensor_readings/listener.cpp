#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <string>

#define gps_topic_name "TODO"
#define costmap_topic_name "/move_base/local_costmap"

// Header
int seq;
int stamp_sec;
int stamp_nsec;
string frame_id;

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
Quaternion orientation;

struct Point {
  float x;
  float y;
  float z;
};
Point position;

int* data = new int[1];

void gpsCallback(const std_msgs::String::ConstPtr& msg) {
  
}

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  // Fill out header
  this->seq = msg->header.seq;
  this->stamp_sec = msg->header.stamp.sec;
  this->stamp_nsec = msg->header.stamp.nsec;
  this->frame_id = msg->header.frame_id;

  // Fill out meta data
  this->load_time_sec = msg->info.map_load_time.sec;
  this->load_time_nsec = msg->info.map_load_time.nsec;
  this->resolution = msg->info.resolution;
  this->width = msg->info.width;
  this->height = msg->info.height;

  this->orientation.x = msg->info.origin.orientation.x;
  this->orientation.y = msg->info.origin.orientation.y;
  this->orientation.z = msg->info.origin.orientation.z;
  this->orientation.w = msg->info.origin.orientation.w;

  this->position.x = msg->info.origin.position.x;
  this->position.y = msg->info.origin.position.y;
  this->position.z = msg->info.origin.position.z;

  delete[] this->data;
  this->data = new int[this->width * this->height];
  for(int i = 0; i < this->height; i++) {
    for(int j = 0; j < this->width; j++) {
      *(this->data + (i * this->width) + j) = msg->data[(i * this->width) + j];
    }
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