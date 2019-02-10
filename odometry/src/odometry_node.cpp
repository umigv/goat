#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("filtered", 1000);
    ros::service::waitForService("gazebo/get_model_state", -1); // timeout?
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    
    gazebo_msgs::GetModelStateRequest model_req;
    model_req.model_name = "mybot";
    gazebo_msgs::GetModelStateResponse model_rep;

    ros::Rate r(50.0); //more than 50 Hz?
    while(nh.ok()) {
        client.call(model_req, model_rep);
        nav_msgs::Odometry odom;
        odom.pose.pose = model_rep.pose;
        odom.twist.twist = model_rep.twist;

        pub.publish(odom);
	    r.sleep();
    }
}
