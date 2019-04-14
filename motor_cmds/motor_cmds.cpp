#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2d.h>
#include<tf/tf.h>
#include <vector>

//Vector pointing in initial starting direction
tf::Vector3 INITIAL_FACING{1,0,0};

tf::Vector3 get_heading(const geometry_msgs::Quaternion& quaternion){
    return tf::Matrix3x3{quaternion} * INITIAL_FACING;
}

void face_towards(const geometry_msgs::Point& target, geometry_msgs::Point& pos, geometry_msgs::Quaternion& orientation, ros::Publisher& pub){
    do{
        tf::Vector3& heading = get_heading(orientation);
        tf::Vector3 target_heading = tf::Vector3{target.x, target.y, target.z} - tf::Vector3{pos.x, pos.y, pos.z};
        double angle = heading.angle(target_heading);
        geometry_msgs::Twist t;
        t.angular.z = angle/scale; // change scale to value
        pub.publish(t);
    } while(angle > threshold || angle < -threshold);
    pub.publish(geometry_msgs::Twist{}); //publish default twist to halt
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