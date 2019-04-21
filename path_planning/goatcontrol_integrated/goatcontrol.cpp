#include "goatcontrol.h"

using namespace std;

// Class for A* algorithm
// Takes in information from sensor readings
// Outputs a vector of coordinate pairs outlining a path from the start position to the target position

class weight_compare
{
GoatControl gc;
friend class GoatControl;

public:
	weight_compare(GoatControl gc): gc(gc) {}
	// pathfinding heuristic as written and described in [insert stanford article source here]
	bool operator()(const position a, const position b)
	{
	double d = std::min(gc.min_cost(a), gc.min_cost(b));
	double weight_a = d * gc.distance(a, gc.target) + gc.cost_map[a.x][a.y];
	double weight_b = d * gc.distance(b, gc.target) + gc.cost_map[b.x][b.y];
	return weight_a < weight_b;
	}
}; // comparator for priority queue

// TO DO: update this GoatControl constructor to reflect its use in the listener node
GoatControl::GoatControl()
{
}

// Breadth first search 
bool GoatControl::make_reachable_collection(std::priority_queue<position, std::vector<position>, weight_compare> &open_set)
{
    while(!open_set.empty())
    {
        position current = open_set.top();
        open_set.pop();

        // if it is the target, exit the loop
        if(current == target)
        {
            return true;
        } // if

        closed_set.insert(current);

        // else add all adjacent states to the queue
	unsigned int x = current.x;
	unsigned int y = current.y;
	position northeast = position(x + 1, y + 1);
    if(cost_map[x + 1][y + 1] < 200 && closed_set.find(northeast) == closed_set.end())
    {
        open_set.push(northeast);
        backtrack_map[x + 1][y + 1] = northeast;
    }

    position east = position(x + 1, y);
    if(cost_map[x + 1][y] < 200 && closed_set.find(east) == closed_set.end())
    {
        open_set.push(east);
        backtrack_map[x + 1][y] = east;
    }
    
    position southeast = position(x + 1, y - 1);
    if(cost_map[x + 1][y - 1]  < 200 && closed_set.find(southeast) == closed_set.end())
    {
        open_set.push(southeast);
        backtrack_map[x + 1][y - 1] = southeast;
    }
    
    position north = position(x, y + 1);
    if(cost_map[x][y + 1] < 200 && closed_set.find(north) == closed_set.end())
    {
        open_set.push(north);
        backtrack_map[x][y + 1] = north;
    }
    
   position south = position(x, y - 1);
    if(cost_map[x][y - 1] < 200 && closed_set.find(south) == closed_set.end())
    {
        open_set.push(south);
        backtrack_map[x][y - 1] = south;
    }
    
    position northwest = position(x - 1, y + 1);
    if(cost_map[x - 1][y + 1] < 200 && closed_set.find(northwest) == closed_set.end())
    {
        open_set.push(northwest);
        backtrack_map[x - 1][y + 1] = northwest;
    }
    
    position west = position(x - 1, y);
    if(cost_map[x - 1][y] < 200 && closed_set.find(west) == closed_set.end())
    {
        open_set.push(west);
        backtrack_map[x - 1][y] = west;
    }
    
    position southwest = position(x - 1, y - 1);
    if(cost_map[x - 1][y - 1] < 200 && closed_set.find(southwest) == closed_set.end())
    {
        open_set.push(southwest);
        backtrack_map[x - 1][y - 1] = southwest;
    }
    } // while

    return false;
} // make_reachable_collection


// Starts at target and backtracks, adding information to the solution path
// Then once it hits the starting position, stops
void GoatControl::backtracker(vector<position> &solution_path)
{
    unsigned int x = target.x;
    unsigned int y = target.y;

    while(x != start.x || y != start.y)
    {
        solution_path.push_back(position(x, y));
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

    if(x > 0 && cost_map[x - 1][y] < min)
    {
        min = cost_map[x - 1][y];
    }

    if(x < cost_map.size() && cost_map[x + 1][y] < min)
    {
        min = cost_map[x + 1][y];
    }

    if(y > 0 && cost_map[x][y - 1] < min)
    {
        min = cost_map[x][y - 1];
    }

    if(y < cost_map[0].size() && cost_map[x][y + 1] < min)
    {
        min = cost_map[x][y + 1];
    }

    if(x > 0 && y > 0 && cost_map[x - 1][y - 1] < min)
    {
        min = cost_map[x - 1][y - 1];
    }

    if(x < cost_map.size() && y < cost_map[0].size() && cost_map[x + 1][y + 1] < min)
    {
        min = cost_map[x + 1][y + 1];
    }

    if(x < cost_map.size() && y > 0 && cost_map[x + 1][y - 1] < min)
    {
        min = cost_map[x + 1][y - 1];
    }

    if(x > 0 && y < cost_map[0].size() && cost_map[x - 1][y + 1] < min)
    {
        min = cost_map[x - 1][y + 1] < min;
    }

    return min;
}
