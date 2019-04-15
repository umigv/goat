#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <string>
#include "a_star.cpp"

#define gps_topic_name "/odometry/filtered"
#define imu_topic_name "/imu/data_jitter_filtered"
#define costmap_topic_name "/move_base/local_costmap/costmap"
#define map_topic_name "/map"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	GoatControl listener;

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
	    GoatControl::position start(start_x, start_y);

	    // Create a position object that holds the position of the target
	    GoatControl::position target(gps_target_x, gps_target_y);

	    // Make a new a_star object
	    //GoatControl pathfinder = a_star(vector<vector<unsigned int>> cost_map_in);

	    // Attempt to find a solution
	    //foundtarget = pathfinder.make_reachable_collection();
	} // while

	vector<GoatControl::position> solution_path;

	// Backtrack from the target and add positions to solution path
	// Solution path is given in reverse order, starting from the goal 
	// and listing the previous position until it reaches the start
	listener.backtracker(solution_path);

	ros::spin();

	return 0;
}
