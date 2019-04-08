#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.>
#include <vector> 

int main(int argc, char** argv){
    ros::init(argc, argv, "motor_cmds"); //good node name?
    ros::nodeHandle nh("~");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    std::vector<geometry_msgs::Point> path = //what A* passes to us, maybe not copy?
    auto it target = path.begin();
    geometry_msgs::Point current_pos = Pose.position //current robot position passed from Sensors
    geometry_msgs::Quaternion current_orientation = Pose.orientation //also passed from Sensors

    double time_step;
    if(!nh.getParam("time_step", time_step)){ //maybe make a default value?
        ROS_FATAL_STREAM("Missing Parameter: time_step");
        ros::shutdown();
        ros::waitForShutdown();
        return 1;
    }

    auto callback = [&path = std::as_const(path), &it, &current_pos = std::as_const(current_pos), 
        &current_orientation = std::as_const(current_orientation), &pub](const ros::TimerEvent& event){
            while(*it != path.end()){
                if(target_ahead(*it, current_pos, current_orientation)){ //watch for edge case of on top of *it
                    face_towards(*it);
                    step();
                    return;
                }
                it++;
            }
            ROS_INFO_STREAM("No More Targets Ahead");
            ros::shutdown();
            ros::waitForShutdown();
            return 1;
        }

    ros::Timer timer = nh.createTimer(ros::Duration(time_step), callback);
    ros::spin();
}