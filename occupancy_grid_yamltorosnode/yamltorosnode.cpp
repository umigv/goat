#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream> 
int main(int argc, char** argv) {
  ros::init(argc, argv, "occupancy_grid_constructor_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<nav_msgs::constructor_node>("occupancy_grid", 1000);
  ros::Rate loop_rate(20);
  int count = 0;
  YAML::Node grid_data = YAML::LoadFIle("../path_planning/occupancy_grid_test_data_generator/occupancyGrid.yaml");
  while (ros::ok()) {
    nav_msgs::OccupancyGrid msg;
    msg.header.seq = grid_data["Header"]["seq"];
    msg.header.stamp = grid_data["Header"]["timeStamp"];
    msg.header.frame_id = grid_data["Header"]["frameID"];
    msg.info.map_load_time = ["MapMetaData"]["mapLoadTime"];
    msg.info.resolution = ["MapMetaData"]["resolution"];
    msg.info.width = ["MapMetaData"]["width"];
    msg.info.height = ["MapMetaData"]["height"];
    msg.origin.position.x = 0;
    msg.origin.position.y = 0;
    msg.origin.position.z = 0;
    msg.origin.orientation.x = 0;
    msg.origin.orientation.y = 0;
    msg.origin.orientation.z = 0;
    msg.origin.orientation.w = 0;
    msg.data = ["Data"];
    pub.publish(msg);
    ROS_INFO("%s", msg.data.c_str());
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
