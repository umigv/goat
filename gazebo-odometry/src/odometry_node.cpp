#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh("~");
    ros::NodeHandle private_handle;
    const int timeout = private_handle.param("timeout", 250);
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odometry/filtered", timeout);
    ros::service::waitForService("gazebo/get_model_state", -1);
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    if(!client.isValid()){
        ROS_ERROR("Service Handle is not valid");
        ros::shutdown(); ros::waitForShutdown(); return 1;
    }

    std::string robot_name;
    if(!private_handle.getParam("robot_name", robot_name)){
        ROS_FATAL_STREAM("Missing Parameter: robot_name");
        ROS_FATAL_STREAM(std::string{argv[1]});
        ros::shutdown(); ros::waitForShutdown(); return 1;
    }
    gazebo_msgs::GetModelStateRequest model_req;
    model_req.model_name = robot_name;
    gazebo_msgs::GetModelStateResponse model_rep;

    auto callback = [&client, &model_req, &model_rep, &pub](const ros::TimerEvent& event){
        if(!client.call(model_req, model_rep)) {
            ROS_ERROR("Call failed to gazebo/get_model_state service");
            ros::shutdown();
        }
        if(model_rep.success) {
            auto odom = boost::make_shared<nav_msgs::Odometry> ();
            odom->pose.pose = model_rep.pose;
            odom->twist.twist = model_rep.twist;

            pub.publish(odom);
        }
        else { // model_rep.success
            ROS_ERROR_STREAM("Call succeded, but request could not be fulfilled: " << model_rep.status_message);
            ros::shutdown();
        }
    };
    const double frequency = private_handle.param("freq", 60.0);
    const ros::Timer timer = nh.createTimer(ros::Duration(1/frequency), callback);
    ros::spin();
}