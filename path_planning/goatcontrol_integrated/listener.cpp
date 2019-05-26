#include "goatcontrol.h"

#define gps_topic_name "/odometry/filtered"
#define imu_topic_name "/imu/data_jitter_filtered"
#define costmap_topic_name "/move_base/local_costmap/costmap"
#define map_topic_name "/map"

using namespace std;

class calculate_path {
private:
	ros::Subscriber &gps;
	ros::Subscriber &costmap;
	GoatControl &listener;
public:
	vector<position> solution_path;
	calculate_path(ros::Subscriber &gps_in, ros::Subscriber &costmap_in, GoatControl &gt_ctrl)
		: gps(gps_in), costmap(costmap_in), listener(gt_ctrl) { }
	void operator()(const ros::TimerEvent&);
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	bool at_target = false;

	GoatControl listener = GoatControl();

	ros::Subscriber gps_sub = n.subscribe(gps_topic_name, 1000, &GoatControl::gpsCallback, &listener);
	ros::Subscriber costmap_sub = n.subscribe(costmap_topic_name, 1000, &GoatControl::costmapCallback, &listener);

	// Calculated path vector can be accessed via path.solution_path
	calculate_path path(gps_sub, costmap_sub);
	
	// Update coordinates once before calculating path
	ros::spinOnce();

	// Start running A* at a frequency specified by period to keep updating path
	// as the area is traversed
	const double period = 0.05;
	ros::Timer timer = n.createTimer(ros::Duration(period), path);
	timer.start();

	// Keep spinning until you reach the target
	while(!at_target)
	{
		ros::spinOnce();
	}

	// Stop running timer that calculates path
	timer.stop();

	// TO DO: publish path once it is calculated

	// TO DO: look into how to determine whether target is reached (at_target
	// currently stays false)

	return 0;
}

void calculate_path::operator()(const ros::TimerEvent&) {
	// Placeholders for the coordinates of the starting position
	unsigned int start_x = listener.start.x;
	unsigned int start_y = listener.start.y;

	// Placeholders for gps coordinates for the target
	unsigned int gps_target_x = listener.target.x;
	unsigned int gps_target_y = listener.target.y;

	// Create a position object that holds the starting position
	position start(start_x, start_y);

	// Create a position object that holds the position of the target
	position target(gps_target_x, gps_target_y);

	// Attempt to find a solution
	bool foundtarget = listener.make_reachable_collection();

	solution_path.clear()
	// Backtrack from the target and add positions to solution path
	// Solution path is given in reverse order, starting from the goal 
	// and listing the previous position until it reaches the start
	listener.backtracker(solution_path);

	// Reverse the solution path so that it goes from the start node to the target
	reverse(solution_path.begin(), solution_path.end());
}