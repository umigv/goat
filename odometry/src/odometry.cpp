#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetModelState.srv>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::nodeHandle nh("~");
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("filtered", 1000);
    ros::Service::waitForService("gazebo/get_model_state", -1)
    ros::ServiceClient client = nh.ServiceClient<gazebo_msgs::ModelState>("gazebo/get_model_state")
    
    gazebo_msgs::ModelState model = GetModelState::GetModelStateRequest()
    model.model_name = "mybot";

    ros::Rate r(50.0);
    while(nh.ok()) {
        srv_model = client.call(model);
        if(model.success) {
            
        }
    }
}