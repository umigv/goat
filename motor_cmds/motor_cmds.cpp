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

int signum_double(double val){
    return (0 < val) - (val < 0);
}

bool point_eq(const geometry_msgs::Point p1, const geometry_msgs::Point p2){
    return p1.x == p2.x && p1.y == p2.y;
}

class Robot{

	private:
		geometry_msgs::Point current_pos; 
    	geometry_msgs::Quaternion current_orientation;
    public:
    	void get_pose_callback(geometry_msgs::Pose msg) {

    		current_pos = msg.position;
    		current_orientation = msg.orientation;
    	}
        tf::Vector3 get_heading(){
            //tf::Matrix3x3 rotation_matrix(current_orientation);
            //return rotation_matrix*kInitial_facing;

            //Use whats passed in
        }   
        double get_angle_to_target(const geometry_msgs::Point& target){
            tf::Vector3 heading = get_heading();
            tf::Vector3 target_heading = tf::Vector3{target.x, target.y, target.z} 
                - tf::Vector3{current_pos.x, current_pos.y, current_pos.z};
            return heading.angle(target_heading);
        }
        bool target_ahead(const geometry_msgs::Point& target){
            if(point_eq(target, current_pos)){
                return false;
            }
            double angle = get_angle_to_target(target);
            return (angle > 0 && angle < M_PI);
        }
        void face_towards(const geometry_msgs::Point& target, ros::Publisher& pub){
            double angle;
            do{
                angle = get_angle_to_target(target);
                geometry_msgs::Twist t;
                t.angular.z = signum_double(angle)*kRotation_magnitude;
                pub.publish(t);
            } while(angle > kAngle_threshold || angle < -1*kAngle_threshold);
            pub.publish(geometry_msgs::Twist{}); //publish default twist to halt
        }
        void step(ros::Publisher& pub){
            geometry_msgs::Twist t;
            const tf::Vector3 norm_heading = get_heading().normalized()*kLinear_magnitude;
            t.linear.x = norm_heading.getX();
            t.linear.y = norm_heading.getY();
            pub.publish(t);
        }  
};

int main(int argc, char** argv){
    ros::init(argc, argv, "motor_cmds"); //good node name?
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    std::vector<geometry_msgs::Point> path;//what A* passes to us, maybe not copy?
    auto target_it = path.begin();

    Robot my_bot;
    ros::Subscriber sub = nh.subscribe("Pose", 1000, &Robot::get_pose_callback, &my_bot); // need to update topic name 

    double time_step;
    if(!nh.getParam("time_step", time_step)){ //maybe make a default value?
        ROS_FATAL_STREAM("Missing Parameter: time_step");
        ros::shutdown();
        ros::waitForShutdown();
        return 1;
    }

    auto callback = [&](const ros::TimerEvent& event){
        while(target_it != path.end()){
            if(my_bot.target_ahead(*target_it)){
                my_bot.face_towards(*target_it, pub);
                my_bot.step(pub); 
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