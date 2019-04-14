#include "a_star.h"

// Class for A* (a_star) algorithm
// Takes in information from sensor readings
// Outputs a vector of coordinate pairs outlining a path from the start position to the target position

// Breadth first search 
bool a_star::make_reachable_collection()
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
void a_star::add_reachable_adjacent(unsigned int x, unsigned int y)
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
void a_star::backtracker(vector<position> &solution_path)
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
double a_star::distance(position a, position b)
{
    double x_diff = a.x - b.x;
    double y_diff = a.y - b.y;
    return sqrt((x_diff * x_diff) + (y_diff * y_diff));
}

// When given a position, calculates the minimum possible cost of traveling from
// said position to any of the adjacent spaces
unsigned int a_star::min_cost(position p) {
    unsigned int min = 0;
    if (p.x > 0) {
        if (costmap.at(p.x-1).at(p.y) < min) {
            min = costmap.at(p.x-1).at(p.y);
        }
    }
    if (p.x < costmap.size()) {
        if (costmap.at(p.x+1).at(p.y) < min) {
            min = costmap.at(p.x+1).at(p.y);
        }
    }
    if (p.y > 0) {
        if (costmap.at(p.x).at(p.y-1) < min) {
            min = costmap.at(p.x).at(p.y-1);
        }
    }
    if (p.y < costmap.at(0).size()) {
        if (costmap.at(p.x).at(p.y+1) < min) {
            min = costmap.at(p.x).at(p.y+1);
        }
    }
    if (p.x > 0 && p.y > 0) {
        if (costmap.at(p.x-1).at(p.y-1) < min) {
            min = costmap.at(p.x-1).at(p.y-1);
        }
    }
    if (p.x < costmap.size() && p.y < costmap.at(0).size()) {
        if (costmap.at(p.x+1).at(p.y+1) < min) {
            min = costmap.at(p.x+1).at(p.y+1);
        }
    }
    if (p.x < costmap.size() && p.y > 0) {
        if (costmap.at(p.x+1).at(p.y-1) < min) {
            min = costmap.at(p.x+1).at(p.y-1);
        }
    }
    if (p.x > 0 && p.y < costmap.at(0).size()) {
        if (costmap.at(p.x-1).at(p.y+1) < min) {
            min = costmap.at(p.x-1).at(p.y+1) < min;
        }
    }

    return min;
}

// Struct that contains an (x, y) pair of coordinates
struct a_star::position
{
    position(x_in, y_in): x(x_in), y(y_in)
    {
    }
    
    unsigned int x;
    unsigned int y;
}; // position struct

// Comparator to take into account cost and distance of two positions
class a_star::weight_compare
{
public:
    bool operator()(position a, position b)
    {
        double d = min(min_cost(a), min_cost(b));
        double weight_a = d * distance(a, target) + costmap.at(a.x).at(a.y);
        double weight_b = d * distance(b, target) + costmap.at(b.x).at(b.y);
        return weight_a < weight_b;
        // TO DO: calculate relative weights of positions according to chosen heuristic
        // and return true if weight(a) < weight(b) and false otherwise
    }
}; // comparator for priority queue