#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <fstream>

// This node publishes /cmd_vel as motor commands and subscribe to /Pose to get feedback
// To run this node, one should type:
// rosrun motor_cmds motor_cmds_node target x position target y position
// the typical template is: rosrun <package> <node> _<param1>:=<value1> _<param2>:=<value2>

// for running simulation on Gazebo - need to run adjust_ori node, and subscribe to /odom instead

int signum_double(double val){
    return (0 < val) - (val < 0);
}



class Robot{
	private:

		geometry_msgs::Point current_pos; 
        double current_theta = 0;

        //std::vector<geometry_msgs::Point>::iterator target_it;
        std::vector<geometry_msgs::Point> path;
        size_t path_size;
        size_t path_index = 0;

        ros::Publisher robot_pub;

        double kAngle_threshold;

        bool point_eq(){

            double delta_x = path.at(path_index).x - current_pos.x;
            double delta_y = path.at(path_index).y - current_pos.y;

            return fabs(delta_x) < 0.01;// && fabs(delta_y) < 0.01;
        }

        double get_angle_to_target(const geometry_msgs::Point& target){

            // theta is angle between itself and x axis (normalize everthing, signs dont matter?)
            // theta(target) - theta(turtle)
            // pos: left rotation
            // neg: right rotation
            //ROS_INFO_STREAM(target.y);
            //ROS_INFO_STREAM(target.x);

            double delta_x = path.at(path_index).x - current_pos.x;
            double delta_y = path.at(path_index).y - current_pos.y;

            double target_theta = atan(delta_y / delta_x);
            //ROS_INFO_STREAM(target_theta - current_theta);
            return target_theta - current_theta;

            // actually no need for exact value - just direction

        }
        bool target_ahead(const geometry_msgs::Point& target){

            if(point_eq()){
                ROS_INFO_STREAM("target is same point as current");
                //++path_index;
                return false;
            }
            return true;
            //double angle = get_angle_to_target(target);
            //ROS_INFO_STREAM("angle to target: ");
            //ROS_INFO_STREAM(angle);
            //return (angle > (-1 * M_PI / 2) && angle < M_PI / 2);
        }

        // double get_target_angle(const geometry_msgs::Point& target) {
        //     return atan(abs(target.y) / abs(target.x));
        // }
    public:

        Robot(ros::NodeHandle& nh, ros::Publisher &pub, std::vector<geometry_msgs::Point> p){
            double initial_heading_x = nh.param("initial_x", 5.544);;
            // if(!nh.getParam("initial_x", initial_heading_x)){
            //     ROS_FATAL_STREAM("Missing Parameter: initial_x");
            //     ros::shutdown();
            //     ros::waitForShutdown();
            // }
            double initial_heading_y = nh.param("initial_y", 5.544);;
            // if(!nh.getParam("initial_y", initial_heading_y)){
            //     ROS_FATAL_STREAM("Missing Parameter: initial_y");
            //     ros::shutdown();
            //     ros::waitForShutdown();
            // }
            double initial_heading_z = nh.param("initial_z", 0);
            // kInitial_heading = tf::Vector3{initial_heading_x, initial_heading_y, initial_heading_z};
            // kRotation_magnitude = nh.param("rotation_magnitude", 10);

            kAngle_threshold = nh.param("angle_threshold", M_PI/128);
            robot_pub = pub;
            path = p;
            path_size = path.size();


            // kLinear_magnitude = nh.param("linear_magnitude", 10);
        }
    	void get_pose_callback(const turtlesim::Pose& msg) {

            // init target_it = path.begin() in ctor
            // find the next ahead target

            while(path_index < path_size) {
                if(target_ahead(path.at(path_index))) {
                    break;
                }
                else {
                    if(++path_index == path_size) {
                        ROS_INFO_STREAM("reached all targets");

                        ros::shutdown();
                        exit(0);
                    }
                }
            }
            
    		current_pos.x = msg.x; // should be before while loop
            current_pos.y = msg.y;
            current_theta = msg.theta;
            ROS_INFO_STREAM("updating pose");
            ROS_INFO_STREAM(current_pos.x);
            ROS_INFO_STREAM(current_pos.y);


            geometry_msgs::Point target_point;
            target_point.x = path.at(path_index).x;
            target_point.y = path.at(path_index).y;

            //face_towards(*target_it);
            //step(*target_it);
            ROS_INFO_STREAM("target x y");
            ROS_INFO_STREAM(target_point.x);
            ROS_INFO_STREAM(target_point.y);

            double delta_x = path.at(path_index).x - current_pos.x;
            double delta_y = path.at(path_index).y - current_pos.y;
            ROS_INFO_STREAM("deltas");
            ROS_INFO_STREAM(delta_x);
            ROS_INFO_STREAM(delta_y);


            // turn first then move forward
            

            double ang_to_target = get_angle_to_target(target_point);

            if(fabs(ang_to_target) > kAngle_threshold) {
                ROS_INFO_STREAM(ang_to_target);
                ROS_INFO_STREAM("-------------------------------------");
                geometry_msgs::Twist t;
                t.angular.z = 1 * signum_double(ang_to_target);
                robot_pub.publish(t);
            }
            else{

                robot_pub.publish(geometry_msgs::Twist{});

                // if x is at target, so is y
                if(fabs(delta_x) > 0.01) {  // || abs(delta_y) > 0.1) {
                    //ROS_INFO_STREAM("why");
                    geometry_msgs::Twist t;
                    t.linear.x = 1 * signum_double(delta_x);
                    robot_pub.publish(t);
                }
                else { // default stall
                    robot_pub.publish(geometry_msgs::Twist{});
                }
            }


           


    		//current_orientation = msg.orientation;
    	}

        
};



int main(int argc, char** argv){
    ros::init(argc, argv, "motor_cmds");
    ros::NodeHandle nh_pub;
    ros::NodeHandle nh_priv("~");

    // for simulation in Gazebo
    ros::Publisher pub = nh_pub.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    // the actual code
    // ros::Publisher pub = nh_pub.advertise<geometry_msgs::Twist>("cmd_vel", 1000);


    std::ifstream in_file(argv[1]);
    if(!in_file.is_open()) {
        ROS_INFO_STREAM("error opening file");
        return 0;
    }

    std::vector<geometry_msgs::Point> fake_path;
    double x, y;

    while(in_file >> x >> y) {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        fake_path.push_back(p);
    }

    // geometry_msgs::Point p1;
    // geometry_msgs::Point p2;
    // ROS_INFO_STREAM(argv[1]);
    // ROS_INFO_STREAM(argv[2]);

    // p1.x = atof(argv[1]);
    // p1.y = atof(argv[2]);
    // p2.x = atof(argv[3]);
    // p2.y = atof(argv[4]);
    // ROS_INFO_STREAM("x and y");
    // ROS_INFO_STREAM(p1.x);
    // ROS_INFO_STREAM(p1.y);

    


    //std::vector<geometry_msgs::Point> path;
    //auto target_it = path.begin();

    Robot my_bot{nh_priv, pub, fake_path};
    // the actual subscriber is to "Pose", which should be published by the sensors team
    //ros::Subscriber sub = nh_pub.subscribe("Pose", 1000, &Robot::get_pose_callback, &my_bot); // need to update topic name 
    //ROS_INFO_STREAM("before sub");
    // for testing on Gazebo, it publishes the current Pose within /Odom
    ros::Subscriber sub = nh_pub.subscribe("turtle1/pose", 1000, &Robot::get_pose_callback, &my_bot);

    //const double time_step = nh_priv.param("time_step", 0.05);
    
    ros::spin();

    //     ROS_INFO_STREAM("No More Targets Aheadddd");
    //     ros::shutdown();
    //     ros::waitForShutdown();
    return 0;
}