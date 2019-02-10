#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odometry/filtered", 1000);
    ros::service::waitForService("gazebo/get_model_state", -1); // timeout?
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    if(!client.isValid()){
        ROS_ERROR("Service Handle is not valid");
        exit(1);
    }
    gazebo_msgs::GetModelStateRequest model_req;
    model_req.model_name = "mybot";
    gazebo_msgs::GetModelStateResponse model_rep;

    ros::Rate r(50.0); //more than 50 Hz?
    while(nh.ok()) {
        if(client.call(model_req, model_rep)) {
            if(model_rep.success) {
                nav_msgs::Odometry odom;
                odom.pose.pose = model_rep.pose;
                odom.twist.twist = model_rep.twist;

                pub.publish(odom);
            }
            else { // model_rep.success
                ROS_ERROR("Call succeded, but returned failed response");
                exit(1);
            }
        }
        else{ // client.call(model_req, model_rep)
            ROS_ERROR("Call failed");
            exit(1);
        }
	    r.sleep();
    }
}
