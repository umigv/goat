#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetModelState.srv>

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::nodeHandle hn("~");
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("filtered", 1000);
    ros::Service::waitForService("gazebo/get_model_state", -1)
    ros::ServiceClient client = nh.ServiceClient<nav_msgs::Odometry>("gazebo/get_model_state")
    
    gazebo_msgs::ModelState model = GetModelState::GetModelStateRequest()
    model.model_name = "mybot";

}