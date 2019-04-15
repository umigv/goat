#include <vector>
#include <priority_queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>

class GoatControl
{
public:
    // Struct that contains an (x, y) pair of coordinates
    struct position;

    // TO DO: update this constructor to reflect it being used in the listener node in a_star.cpp
    GoatControl(position str, position tar, vector<vector<unsigned int>> &c_m);

    // Breadth first search 
    bool make_reachable_collection();

    // Returns whether the position is reachable or not
    void add_reachable_adjacent(unsigned int x, unsigned int y);

    // Starts at target and backtracks, adding information to the solution path
    // Then once it hits the starting position, stops and reverses the solution path vector
    void backtracker(vector<position> &solution_path);

    // Calculates Euclidean distance between two positions
    double distance(position a, position b);

    // When given a position, calculates the minimum possible cost of traveling from
    // said position to any of the adjacent spaces
    unsigned int min_cost(position p);

    // Comparator to take into account cost and distance of two positions
    class weight_compare;

    void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

private:

    // create a position object that holds the starting position
    position start;

    // create a position object that holds the position of the target
    position target;

    // priority queue to order current positions and eventually find the target
    priority_queue<position, weight_compare> open_set;

    // unordered set to store the positions that have already been visited
    unordered_set<position> closed_set;

    // cost map from input that stores the values of the costs of each location
    vector<vector<unsigned int>> cost_map;

    // data structure for backtracking
    vector<vector<position>> backtrack_map;

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