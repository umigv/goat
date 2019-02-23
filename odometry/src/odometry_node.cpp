#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odometry/filtered", 250);
    ros::service::waitForService("gazebo/get_model_state", -1); // timeout?
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    if(!client.isValid()){
        ROS_ERROR("Service Handle is not valid");
        ros::shutdown(); ros::waitForShutdown(); return 1;
    }
    gazebo_msgs::GetModelStateRequest model_req;
    model_req.model_name = "mybot";
    gazebo_msgs::GetModelStateResponse model_rep;

    auto callback = [&client, &model_req, &model_rep, &pub](const ros::TimerEvent& event){
        if(!client.call(model_req, model_rep)) {
            ROS_ERROR("Call failed to gazebo/get_model_state service");
            ros::shutdown(); ros::waitForShutdown(); return 1;
        }
        if(model_rep.success) {
            boost::shared_ptr<nav_msgs::Odometry> odom = boost::make_shared<nav_msgs::Odometry> ();
            odom->pose.pose = model_rep.pose;
            odom->twist.twist = model_rep.twist;

            pub.publish(odom);
        }
        else { // model_rep.success
            ROS_ERROR("Call succeded, but returned failed response");
            ROS_ERROR_STREAM(model_rep.status_message);
            ros::shutdown(); ros::waitForShutdown(); return 1;
        }
    };

    ros::Timer frequency = nh.createTimer(ros::Duration(1/50), callback);
    ros::spin();
}