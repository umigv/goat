#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetModelState.srv>

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::nodeHandle nh("~");
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("filtered", 1000);
    ros::Service::waitForService("gazebo/get_model_state", -1) // timeout?
    ros::ServiceClient client = nh.ServiceClient<gazebo_msgs::ModelState>("gazebo/get_model_state")
    
    gazebo_msgs::ModelState model = GetModelState::GetModelStateRequest()
    model.model_name = "mybot";

    ros::Rate r(50.0); //more than 50 Hz?
    while(nh.ok()) {
        srv_model = client.call(model);
        if(model.success) {
            nav_msgs::Odometry odom(); //ctors?
            odom.pose.pose = model.Pose;
            odom.twist.twist = model.twist;

            pub.publish(odom);
        }
        // exception handling?
    }
}