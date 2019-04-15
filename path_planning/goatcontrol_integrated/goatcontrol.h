#ifndef GOATCONTROL_H_
#define GOATCONTROL_H_

#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>

    // Struct that contains an (x, y) pair of coordinates
	struct position
	{
	    position(unsigned int x_in, unsigned int y_in): x(x_in), y(y_in)
	    {
	    }
	    
	    unsigned int x;
	    unsigned int y;
	}; // position struct

    // Comparator to take into account cost and distance of two positions
	class weight_compare
	{
	public:
		weight_compare(GoatControl gc) {}
	    // pathfinding heuristic as written and described in [insert stanford article source here]
	    bool operator()(const position a, const position b)
	    {
		double d = std::min(gc.min_cost(a), gc.min_cost(b));
		double weight_a = d * gc.distance(a, gc.target) + gc.cost_map[a.x][a.y];
		double weight_b = d * gc.distance(b, gc.target) + gc.cost_map[b.x][b.y];
		return weight_a < weight_b;
	    }
	}; // comparator for priority queue

class GoatControl
{
public:
	// TO DO: update this GoatControl constructor to reflect its use in the listener node
	GoatControl();

    // Breadth first search 
    bool make_reachable_collection();

    // Returns whether the position is reachable or not
    void add_reachable_adjacent(unsigned int x, unsigned int y);

    // Starts at target and backtracks, adding information to the solution path
    // Then once it hits the starting position, stops and reverses the solution path vector
    void backtracker(std::vector<position> &solution_path);

    // Calculates Euclidean distance between two positions
    double distance(position a, position b);

    // When given a position, calculates the minimum possible cost of traveling from
    // said position to any of the adjacent spaces
    unsigned int min_cost(position p);

    void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

private:

    // create a position object that holds the starting position
    position start;

    // create a position object that holds the position of the target
    position target;

    // priority queue to order current positions and eventually find the target
    std::priority_queue<position, std::vector<position>, weight_compare> open_set;

    // unordered set to store the positions that have already been visited
    std::unordered_set<position> closed_set;

    // cost map from input that stores the values of the costs of each location
    std::vector<std::vector<unsigned int>> cost_map;

    // data structure for backtracking
    std::vector<std::vector<position>> backtrack_map;

    int costmap_width;
    int costmap_height;

    friend struct position;
    friend class weight_compare;

    /*
    struct Header
    {
        int seq;
        int stamp_sec;
        int stamp_nsec;
        string frame_id;
    };
    Header costmapHeader;

    //Map meta data
    int load_time_sec;
    int load_time_nsec;
    float resolution;

    struct Quaternion
    {
        float x;
        float y;
        float z;
        float w;
    };

    struct Point
    {
        float x;
        float y;
        float z;
    };

    struct Vector3
    {
        float x;
        float y;
        float z;
    };

    struct Twist
    {
        Vector3 linear;
        Vector3 angular;
    };

    struct Pose
    {
        Point position;
        Quaternion orientation;
    };

    Pose costMapPose;

    int* data = new int[1];

    // GPS
    Header gpsHeader;
    Pose gpsPose;
    float gpsPoseCovariance[36];
    Twist gpsTwist;
    float gpsTwistCovariance[36];
    */
};
#endif
