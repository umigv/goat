#include "a_star.h"

// Class for A* (a_star) algorithm
// Takes in information from sensor readings
// Outputs a vector of coordinate pairs outlining a path from the start position to the target position

// TO DO: update this GoatControl constructor to reflect its use in the listener node
GoatControl::GoatControl(position str, position tar, vector<vector<unsigned int>> &c_m): start(str), target(tar), cost_map(c_m)
{
    // Add the starting position to the priority queue to begin
    open_set.push(start);

    // Resize the backtrack_map to the size of the costmap
    backtrack_map.resize(cost_map.size(), vector<position>(cost_map[0].size()));
}

// Breadth first search 
bool GoatControl::make_reachable_collection()
{
    while(!open_set.empty())
    {
        position current = open_set.front();
        open_set.pop_front();

        // if it is the target, exit the loop
        if(current == target)
        {
            return true;
        } // if

        closed_set.insert(current);

        // else add all adjacent states to the queue
        add_reachable_adjacent(current.x, current.y);
    } // while

    return false;
} // make_reachable_collection

// Returns whether the position is reachable or not
void GoatControl::add_reachable_adjacent(unsigned int x, unsigned int y)
{
    // If the position contains an obstacle or is very close to an obstacle, don't add it
    // also don't add to the queue if the position is already in the closed set (already visited)
    // add to the backtrack_map the original position (where each of the neighbors came from)
    
    position northeast = position(x + 1, y + 1);
    if(cost_map(x + 1, y + 1) < 200 && closed_set.find(northeast) == closed_set.end())
    {
        open_set.push(northeast);
        backtrack_map[x + 1][y + 1] = northeast;
    }

    position east = position(x + 1, y);
    if(cost_map(x + 1, y) < 200 && closed_set.find(east) == closed_set.end())
    {
        open_set.push(east);
        backtrack_map[x + 1][y] = east;
    }
    
    position southeast = position(x + 1, y - 1);
    if(cost_map(x + 1, y - 1)  < 200 && closed_set.find(southeast) == closed_set.end())
    {
        open_set.push(southeast);
        backtrack_map[x + 1][y - 1] = southeast;
    }
    
    position north = position(x, y + 1);
    if(cost_map(x, y + 1)  < 200 && closed_set.find(north) == closed_set.end())
    {
        open_set.push(north);
        backtrack_map[x][y + 1] = north;
    }
    
    position south = position(x, y - 1);
    if(cost_map(x, y - 1)  < 200 && closed_set.find(south) == closed_set.end())
    {
        open_set.push(south);
        backtrack_map[x][y - 1] = south;
    }
    
    position northwest = position(x - 1, y + 1);
    if(cost_map(x - 1, y + 1)  < 200 && closed_set.find(northwest) == closed_set.end())
    {
        open_set.push(northwest);
        backtrack_map[x - 1][y + 1] = northwest;
    }
    
    position west = position(x - 1, y);
    if(cost_map(x - 1, y)  < 200 && closed_set.find(west) == closed_set.end())
    {
        open_set.push(west);
        backtrack_map[x - 1][y] = west;
    }
    
    position southwest = position(x - 1, y - 1);
    if(cost_map(x - 1, y - 1)  < 200 && closed_set.find(southwest) == closed_set.end())
    {
        open_set.push(southwest);
        backtrack_map[x - 1][y - 1] = southwest;
    }
} // add_reachable_adjacent

// Starts at target and backtracks, adding information to the solution path
// Then once it hits the starting position, stops
void GoatControl::backtracker(vector<position> &solution_path)
{
    unsigned int x = target.x;
    unsigned int y = target.y;

    while(x != start.x || y != start.y)
    {
        solution_path.push(position(x, y));
        position before = backtrack_map[x][y];
        x = before.x;
        y = before.y;
    }
} // backtracker

// Calculates Euclidean distance between two positions
double GoatControl::distance(position a, position b)
{
    double x_diff = a.x - b.x;
    double y_diff = a.y - b.y;
    return sqrt((x_diff * x_diff) + (y_diff * y_diff));
}

// When given a position, calculates the minimum possible cost of traveling from
// said position to any of the adjacent spaces
unsigned int GoatControl::min_cost(position p)
{
    unsigned int min = 0;
    unsigned int x = p.x;
    unsigned int y = p.y;

    if(x > 0 && costmap[x - 1][y] < min)
    {
        min = costmap[x - 1][y];
    }

    if(x < costmap.size() && costmap[x + 1][y] < min)
    {
        min = costmap[x + 1][y];
    }

    if(y > 0 && costmap[x][y - 1] < min)
    {
        min = costmap[x][y - 1];
    }

    if(y < costmap[0].size() && costmap[x][y + 1] < min)
    {
        min = costmap[x][y + 1];
    }

    if(x > 0 && y > 0 && costmap[x - 1][y - 1] < min)
    {
        min = costmap[x - 1][y - 1];
    }

    if(x < costmap.size() && y < costmap[0].size() && costmap[x + 1][y + 1] < min)
    {
        min = costmap[x + 1][y + 1];
    }

    if(x < costmap.size() && y > 0 && costmap[x + 1][y - 1] < min)
    {
        min = costmap[x + 1][y - 1];
    }

    if(x > 0 && y < costmap[0].size() && costmap[x - 1][y + 1] < min)
    {
        min = costmap[x - 1][y + 1] < min;
    }

    return min;
}

// Struct that contains an (x, y) pair of coordinates
struct GoatControl::position
{
    position(x_in, y_in): x(x_in), y(y_in)
    {
    }
    
    unsigned int x;
    unsigned int y;
}; // position struct

// Comparator to take into account cost and distance of two positions
class GoatControl::weight_compare
{
public:
    // pathfinding heuristic as written and described in [insert stanford article source here]
    bool operator()(position a, position b)
    {
        double d = min(min_cost(a), min_cost(b));
        double weight_a = d * distance(a, target) + costmap[a.x][a.y];
        double weight_b = d * distance(b, target) + costmap[b.x][b.y];
        return weight_a < weight_b;
    }
}; // comparator for priority queue

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
    this->target = position(msg->pose.pose.position.x, msg->pose.pose.position.y);

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