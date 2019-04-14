#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include<tf/tf.h>
#include <vector>
#include <math.h>

//Vector pointing in initial starting direction
const tf::Vector3 kInitial_facing{1,0,0};
//Magnitude of our left/right rotation vector
const double kRotation_magnitude = 10;
//Threshold for angle facing in radians
const double kAngle_threshold = M_PI/8;
//Magnitude of our linear velocity vector
const double kLinear_magnitude = 10;

double get_angle_to_target(const geometry_msgs::Point& target, geometry_msgs::Point& pos, geometry_msgs::Quaternion& orientation){
    tf::Vector3& heading = get_heading(orientation);
    tf::Vector3 target_heading = tf::Vector3{target.x, target.y, target.z} - tf::Vector3{pos.x, pos.y, pos.z};
    return heading.angle(target_heading);
}

bool target_ahead(const geometry_msgs::Point& target, geometry_msgs::Point& pos, geometry_msgs::Quaternion& orientation){
    if(target == pos){
        return false;
    }
    double angle = get_angle_to_target(target, pos, orientation);
    return (angle > 0 && angle < M_PI)
}

tf::Vector3 get_heading(const geometry_msgs::Quaternion& quaternion){
    tf::Matrix3x3 rotation_matrix{quaternion};
    return rotation_matrix*kInitial_facing;
}

template <typename t>
int signum(T val){
    return (T(0) < val) - (val < T(0));
}

void face_towards(const geometry_msgs::Point& target, geometry_msgs::Point& pos, geometry_msgs::Quaternion& orientation, ros::Publisher& pub){
    do{
        double angle = get_angle_to_target(target, pos, orientation);
        geometry_msgs::Twist t;
        t.angular.z = signum(angle)*kRotation_magnitude;
        pub.publish(t);
    } while(angle > kAngle_threshold || angle < -1*kAngle_threshold);
    pub.publish(geometry_msgs::Twist{}); //publish default twist to halt
}

void step(geometry_msgs::Quaternion& orientation, ros::Publisher& pub){
    geometry_msgs::Twist t;
    const tf::Vector3 norm_heading = get_heading(orientation).normalized()*kLinear_magnitude;
    t.linear.x = norm_heading.getX();
    t.linear.y = norm_heading.getY();
    pub.publish(t);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "motor_cmds"); //good node name?
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    std::vector<geometry_msgs::Point> path;//what A* passes to us, maybe not copy?
    auto target_it = path.begin();
    geometry_msgs::Point current_pos; 
    geometry_msgs::Quaternion current_orientation;

    // callback can not take params, it has to be in main unless created as a class member function
    void callback_getPose = [&current_pos, &current_orientation](geometry_msgs::Pose msg) {
        current_pos = msg.position;
        current_orientation = msg.orientation;
    };
    ros::Subscriber sub = nh.subscribe("Pose", 1000, callback_getPose); // need to update topic name 

    double time_step;
    if(!nh.getParam("time_step", time_step)){ //maybe make a default value?
        ROS_FATAL_STREAM("Missing Parameter: time_step");
        ros::shutdown();
        ros::waitForShutdown();
        return 1;
    }

    auto callback = [&](const ros::TimerEvent& event){
            while(target_it != path.end()){
                if(target_ahead(*target_it, current_pos, current_orientation)){
                    face_towards(*target_it, current_pos, current_orientation, pub);
                    step(current_orientation, pub); 
                }
                else{
                    target_it++;
                }
            }
            ROS_INFO_STREAM("No More Targets Ahead");
            ros::shutdown();
            ros::waitForShutdown();
            return 1;
        };
    while(ros::ok()){
        ros::Timer timer = nh.createTimer(ros::Duration(time_step), callback);
        ros::spin();
    }
}