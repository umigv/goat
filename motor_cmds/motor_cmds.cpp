#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <vector>
#include <math.h>

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

        //Vector pointing in initial starting direction
        tf::Vector3 kInitial_heading;
        //Magnitude of our left/right rotation vector
        double kRotation_magnitude;
        //Threshold for angle facing in radians
        double kAngle_threshold;
        //Magnitude of our linear velocity vector
        double kLinear_magnitude;

        tf::Vector3 get_heading(){
            tf::Quaternion q(current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w);
            if(*q == 0){
                return kInitial_heading;
            }
            tf::Matrix3x3 rotation_matrix(q);
            return rotation_matrix*kInitial_heading;
        }
        double get_angle_to_target(const geometry_msgs::Point& target){
            tf::Vector3 heading = get_heading();
            if(*heading == 0){
                return 0;
            }
            tf::Vector3 target_heading = tf::Vector3{target.x, target.y, target.z} 
                - tf::Vector3{current_pos.x, current_pos.y, current_pos.z};
            return heading.angle(target_heading);
        }
    public:
        Robot(ros::NodeHandle& nh){
            double initial_heading_x;
            if(!nh.getParam("initial_x", initial_heading_x)){
                ROS_FATAL_STREAM("Missing Parameter: initial_x");
                ros::shutdown();
                ros::waitForShutdown();
            }
            double initial_heading_y;
            if(!nh.getParam("initial_y", initial_heading_y)){
                ROS_FATAL_STREAM("Missing Parameter: initial_y");
                ros::shutdown();
                ros::waitForShutdown();
            }
            double initial_heading_z = nh.param("initial_z", 0);
            kInitial_heading = tf::Vector3{initial_heading_x, initial_heading_y, initial_heading_z};
            kRotation_magnitude = nh.param("rotation_magnitude", 10);
            kAngle_threshold = nh.param("angle_threshold", M_PI/16);
            kLinear_magnitude = nh.param("linear_magnitude", 10);
        }
    	void get_pose_callback(geometry_msgs::Pose& msg) {

    		current_pos = msg.position;
    		current_orientation = msg.orientation;
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
    ros::init(argc, argv, "motor_cmds");
    ros::NodeHandle nh_pub;
    ros::NodeHandle nh_priv("~");
    ros::Publisher pub = nh_pub.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    p2.x = 10;
    p2.y = 10;
    geometry_msgs::Point p3;
    p3.x = 20;
    p3.y = 15;
    std::vector<geometry_msgs::Point> path{p1, p2, p3};
    auto target_it = path.begin();

    Robot my_bot{nh_priv};
    ros::Subscriber sub = nh_pub.subscribe("Pose", 1000, &Robot::get_pose_callback, &my_bot); // need to update topic name 

    const double time_step = nh_priv.param("time_step", 0.05);
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
    };
    while(ros::ok()){
        ros::Timer timer = nh_priv.createTimer(ros::Duration(time_step), callback);
        ros::spin();
    }
}