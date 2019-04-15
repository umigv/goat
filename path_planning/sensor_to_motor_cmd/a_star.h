#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>

class GoatControl
{
public:
    // Struct that contains an (x, y) pair of coordinates
	struct GoatControl::position
	{
	    GoatControl::position(x_in, y_in): x(x_in), y(y_in)
	    {
	    }
	    
	    unsigned int x;
	    unsigned int y;
	}; // position struct

    // TO DO: update this constructor to reflect it being used in the listener node in a_star.cpp
    GoatControl(GoatControl::position str, GoatControl::position tar, std::vector<std::vector<unsigned int>> &c_m);

    // Breadth first search 
    bool make_reachable_collection();

    // Returns whether the position is reachable or not
    void add_reachable_adjacent(unsigned int x, unsigned int y);

    // Starts at target and backtracks, adding information to the solution path
    // Then once it hits the starting position, stops and reverses the solution path vector
    void backtracker(std::vector<position> &solution_path);

    // Calculates Euclidean distance between two positions
    double distance(GoatControl::position a, GoatControl::position b);

    // When given a position, calculates the minimum possible cost of traveling from
    // said position to any of the adjacent spaces
    unsigned int min_cost(GoatControl::position p);

    // Comparator to take into account cost and distance of two positions
	class GoatControl::weight_compare
	{
	public:
	    // pathfinding heuristic as written and described in [insert stanford article source here]
	    bool operator()(GoatControl::position a, GoatControl::position b)
	    {
		double d = min(min_cost(a), min_cost(b));
		double weight_a = d * distance(a, target) + costmap[a.x][a.y];
		double weight_b = d * distance(b, target) + costmap[b.x][b.y];
		return weight_a < weight_b;
	    }
	}; // comparator for priority queue

    void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

private:

    // create a position object that holds the starting position
    GoatControl::position start;

    // create a position object that holds the position of the target
    GoatControl::position target;

    // priority queue to order current positions and eventually find the target
    std::priority_queue<GoatControl::position, weight_compare> open_set;

    // unordered set to store the positions that have already been visited
    std::unordered_set<GoatControl::position> closed_set;

    // cost map from input that stores the values of the costs of each location
    std::vector<std::vector<unsigned int>> cost_map;

    // data structure for backtracking
    std::vector<std::vector<GoatControl::position>> backtrack_map;

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
