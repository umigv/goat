#include "goatcontrol.h"

#define gps_topic_name "/odometry/filtered"
#define imu_topic_name "/imu/data_jitter_filtered"
#define costmap_topic_name "/move_base/local_costmap/costmap"
#define map_topic_name "/map"

using namespace std;

position GoatControl::target = position();
std::vector<std::vector<unsigned int> > GoatControl::cost_map;

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

	// TO DO: implement the following logic described in the comment
	
	// When one of the current paths exceeds the width or height of the current costmap,
	// reload the costmap data to update and extend the best current path
	// After choosing best path, backtrack and move to it using motor commands
	// Then clear member variables of goatcontrol object and start over with current position
	// Keep looping until you reach the target
	while(!foundtarget || !reachedBound)
	{
	    // Create a position object that holds the starting position
	    position start(start_x, start_y);

	    // Create a position object that holds the position of the target
	    position target(gps_target_x, gps_target_y);

	    // Attempt to find a solution
	    foundtarget = listener.make_reachable_collection();
	} // while

	vector<position> solution_path;

	// Backtrack from the target and add positions to solution path
	// Solution path is given in reverse order, starting from the goal 
	// and listing the previous position until it reaches the start
	
	listener.backtracker(solution_path);

	// Reverse the solution path so that it goes from the start node to the target
	reverse(solution_path.begin(), solution_path.end());

	ros::spin();

	return 0;
}
