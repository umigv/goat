#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include<tf/tf.h>
#include <vector>
#include <math.h>

//Vector pointing in initial starting direction
const tf::Vector3 INITIAL_FACING{1,0,0};
//Magnitude of our left/right rotation vector
const double ROTATION_MAGNITUDE = 10;
//Threshold for angle facing in radians
const double ANGLE_THRESHOLD = M_PI/8;
//Magnitude of our linear velocity vector
const double LINEAR_MAGNITUDE = 10;

double get_angle_to_target(const geometry_msgs::Point& target, geometry_msgs::Point& pos, geometry_msgs::Quaternion& orientation){
    tf::Vector3& heading = get_heading(orientation);
    tf::Vector3 target_heading = tf::Vector3{target.x, target.y, target.z} - tf::Vector3{pos.x, pos.y, pos.z};
    return heading.angle(target_heading);
}

bool target_ahead(const geometry_msgs::Point& target, geometry_msgs::Point& pos, geometry_msgs::Quaternion& orientation){
    double angle = get_angle_to_target(target, pos, orientation);
    return (angle > 0 && angle < M_PI)
}

tf::Vector3 get_heading(const geometry_msgs::Quaternion& quaternion){
    return tf::Matrix3x3{quaternion}*INITIAL_FACING;
}
template <typename t>
int signum(T val){
    return (T(0) < val) - (val < T(0));
}

void face_towards(const geometry_msgs::Point& target, geometry_msgs::Point& pos, geometry_msgs::Quaternion& orientation, ros::Publisher& pub){
    do{
        double angle = get_angle_to_target(target, pos, orientation);
        geometry_msgs::Twist t;
        t.angular.z = signum(angle)*ROTATION_MAGNITUDE;
        pub.publish(t);
    } while(angle > threshold || angle < -1*threshold);
    pub.publish(geometry_msgs::Twist{}); //publish default twist to halt
}

void step(geometry_msgs::Quaternion& orientation, ros::Publisher& pub){
    geometry_msgs::Twist t;
    t.linear = get_heading(orientation).normalized()*LINEAR_MAGNITUDE;
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