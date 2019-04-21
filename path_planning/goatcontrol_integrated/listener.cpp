#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <string>
#include "goatcontrol.h"

#define gps_topic_name "/odometry/filtered"
#define imu_topic_name "/imu/data_jitter_filtered"
#define costmap_topic_name "/move_base/local_costmap/costmap"
#define map_topic_name "/map"

using namespace std;

void GoatControl::gpsCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    /*
    this->gpsHeader.seq = msg->header.seq;
    this->gpsHeader.stamp_sec = msg->header.stamp.sec;
    this->gpsHeader.stamp_nsec = msg->header.stamp.nsec;
    this->gpsHeader.frame_id = msg->header.frame_id;

    this->gpsPose.position.x = msg->pose.pose.position.x;
    this->gpsPose.position.y = msg->pose.pose.position.y;
    this->gpsPose.position.z = msg->pose.pose.position.z;

    this->gpsPose.orientation.x = msg->pose.pose.orientation.x;
    this->gpsPose.orientation.y = msg->pose.pose.orientation.y;
    this->gpsPose.orientation.z = msg->pose.pose.orientation.z;
    this->gpsPose.orientation.w = msg->pose.pose.orientation.w;

    for(int i = 0; i < 36; i++)
    {
        this->gpsPoseCovariance[i] = msg->pose.covariance[i];
        this->gpsTwistCovariance[i] = msg->twist.covariance[i];
    }

    this->gpsTwist.linear.x = msg->twist.twist.linear.x;
    this->gpsTwist.linear.y = msg->twist.twist.linear.y;
    this->gpsTwist.linear.z = msg->twist.twist.linear.z;

    this->gpsTwist.angular.x = msg->twist.twist.angular.x;
    this->gpsTwist.angular.y = msg->twist.twist.angular.y;
    this->gpsTwist.angular.z = msg->twist.twist.angular.z;
    */

    // Fill out the needed "target" position member variable using info from the odom message
    target = position(msg->pose.pose.position.x, msg->pose.pose.position.y);

}

void GoatControl::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    /*
    // Fill out header
    this->costmapHeader.seq = msg->header.seq;
    this->costmapHeader.stamp_sec = msg->header.stamp.sec;
    this->costmapHeader.stamp_nsec = msg->header.stamp.nsec;
    this->costmapHeader.frame_id = msg->header.frame_id;

    // Fill out meta data
    this->load_time_sec = msg->info.map_load_time.sec;
    this->load_time_nsec = msg->info.map_load_time.nsec;
    this->resolution = msg->info.resolution;
    this->width = msg->info.width;
    this->height = msg->info.height;

    this->costMapPose.orientation.x = msg->info.origin.orientation.x;
    this->costMapPose.orientation.y = msg->info.origin.orientation.y;
    this->costMapPose.orientation.z = msg->info.origin.orientation.z;
    this->costMapPose.orientation.w = msg->info.origin.orientation.w;

    this->costMapPose.position.x = msg->info.origin.position.x;
    this->costMapPose.position.y = msg->info.origin.position.y;
    this->costMapPose.position.z = msg->info.origin.position.z;

    delete[] this->data;
    this->data = new int[this->width * this->height];

    for(int i = 0; i < (this->width * this->height); i++)
    {
        this->data[i] = msg->data[i];
    }
    */

    // Fill out the costmap width and height from the occupancy grid info message
    this->costmap_width = msg->info.width;
    this->costmap_height = msg->info.height;

    // Fill out the needed "start" position member variable using info from the costmap origin message
    this->start = position(msg->info.origin.orientation.x, msg->info.origin.orientation.y);

    // Fill out the costmap member variable using info from the occupancy grid costmap message
    for(int i = 0; i < this->costmap_width; i++)
    {
        for(int j = 0; j < this->costmap_height; j++)
        {
            this->cost_map[i][j] = msg->data[i + (j * costmap_width)];
        }
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	GoatControl listener = GoatControl();

	ros::Subscriber gps_sub = n.subscribe(gps_topic_name, 1000, &GoatControl::gpsCallback, &listener);
	ros::Subscriber costmap_sub = n.subscribe(costmap_topic_name, 1000, &GoatControl::costmapCallback, &listener);

	// Placeholders for the coordinates of the starting position
	unsigned int start_x = 0;
	unsigned int start_y = 0;

	// Placeholders for gps coordinates for the target
	unsigned int gps_target_x = 0;
	unsigned int gps_target_y = 0;

	bool foundtarget = false;

	// TO DO: figure out whether this loop should be done in a callback function
	// and how the timing of the callback functions and this control loop will interact
	
	// Loop while a solution isn't found
	while(!foundtarget)
	{
	    // Create a position object that holds the starting position
	    position start(start_x, start_y);

	    // Create a position object that holds the position of the target
	    position target(gps_target_x, gps_target_y);

	    // Attempt to find a solution
	    // foundtarget = listener.make_reachable_collection(open_set);
	} // while

	vector<position> solution_path;

	// Backtrack from the target and add positions to solution path
	// Solution path is given in reverse order, starting from the goal 
	// and listing the previous position until it reaches the start
	
	listener.backtracker(solution_path);

	ros::spin();

	return 0;
}
