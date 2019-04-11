#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2d.h>
#include<tf/tf.h>
#include <vector>

tf::Vector3 get_heading(const geometry_msgs::Quaternion& quaternion){
    tf::Matrix3x3 rotation_matrix{quaternion};
    //TODO get heading vector from quaternion
    return yaw;
}

void face_towards(const geometry_msgs::Point& target, geometry_msgs::Point& pos, geometry_msgs::Quaternion& orientation, ros::Publisher& pub){
    double angle = 100; // change to constant
    while(angle > threshold || angle < -threshold){
        tf::Vector3& heading = quaternion_to_yaw(orientation);
        tf::Vector3 target_heading = tf::Vector3{target.x, target.y, target.z} - tf::Vector3{pos.x, pos.y, pos.z};
        angle = heading.angle(target_heading);
        geometry_msgs::Twist t;
        t.angular.z = angle/scale; // change scale to value
        pub.publish(t);
    }
    pub.publish(geometry_msgs::Twist{});
}

void step(geometry_msgs::Quaternion& orientation, ros::Publisher& pub){
    geometry_msgs::Twist t;
    t.linear = get_heading(orientation);
    pub.publish(t);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "motor_cmds"); //good node name?
    ros::nodeHandle nh("~");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subsriber sub = nh.subscribe("Pose", 1000, callback_getPose); // need to update topic name 

    std::vector<geometry_msgs::Point> path = //what A* passes to us, maybe not copy?
    auto it target = path.begin();
    geometry_msgs::Point current_pos; //current robot position passed from Sensors
    geometry_msgs::Quaternion current_orientation; //also passed from Sensors

    // callback can not take params, it has to be in main unless created as a class member function
    void callback_getPose(geometry_msgs::Pose msg) {
        current_pos = msg.position;
        current_orientation = msg.orientation;
    }  

    double time_step;
    if(!nh.getParam("time_step", time_step)){ //maybe make a default value?
        ROS_FATAL_STREAM("Missing Parameter: time_step");
        ros::shutdown();
        ros::waitForShutdown();
        return 1;
    }

    auto callback = [&path = std::as_const(path), &it, &current_pos, 
        &current_orientation, &pub](const ros::TimerEvent& event){
            while(*it != path.end()){
                if(target_ahead(*it, current_pos, current_orientation)){ //watch for edge case of on top of *it
                    face_towards(*it, current_pos, current_orientation, pub);
                    step(current_orientation, pub);
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