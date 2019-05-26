#include "a_star_test.h"

using namespace std;

// Class for A* algorithm
// Takes in information from sensor readings
// Outputs a vector of coordinate pairs outlining a path from the start position to the target position

// Fill costmap and its width/height and starting and target positions with values for testing
a_star_test::a_star_test(int width, int height, position strt, position trg, std::vector<std::vector<unsigned int> > costs)
{
    costmap_width = width;
    costmap_height = height;
    start = strt;
    target = trg;
    cost_map = costs;
}

// Breadth first search 
bool a_star_test::make_reachable_collection()
{
    backtrack_map.resize(costmap_height,
                         std::vector<position>(costmap_width, position()));
    
    open_set.push(start);
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

		// add southeast state
    	position southeast = position(x + 1, y + 1);
    	// but only if it is within the bounds of the cost_map,
    	// reachable (cost_map val < 200), and it hasn't been added already
	    if((x + 1 < costmap_height) && (y + 1 < costmap_width)
           && cost_map[x + 1][y + 1] < 200
           && closed_set.find(southeast) == closed_set.end())
	    {
			open_set.push(southeast);
            closed_set.insert(southeast);
			backtrack_map[x + 1][y + 1] = current;
	    }

		position east = position(x + 1, y);
		if((x + 1 < costmap_height) && cost_map[x + 1][y] < 200
           && closed_set.find(east) == closed_set.end())
		{
			open_set.push(east);
            closed_set.insert(east);
			backtrack_map[x + 1][y] = current;
		}

		position northeast = position(x + 1, y - 1);
		if((x + 1 < costmap_height) && y >= 1 && cost_map[x + 1][y - 1]  < 200
           && closed_set.find(northeast) == closed_set.end())
		{
			open_set.push(northeast);
            closed_set.insert(northeast);
			backtrack_map[x + 1][y - 1] = current;
		}

		position south = position(x, y + 1);
		if((y + 1 < costmap_width) && cost_map[x][y + 1] < 200
           && closed_set.find(south) == closed_set.end())
		{
			open_set.push(south);
            closed_set.insert(south);
			backtrack_map[x][y + 1] = current;
		}

		position north = position(x, y - 1);
		if(y >= 1 && cost_map[x][y - 1] < 200
           && closed_set.find(north) == closed_set.end())
		{
			open_set.push(north);
            closed_set.insert(north);
			backtrack_map[x][y - 1] = current;
		}

		position southwest = position(x - 1, y + 1);
		if(x >= 1 && (y + 1 < costmap_width) && cost_map[x - 1][y + 1] < 200
           && closed_set.find(southwest) == closed_set.end())
		{
			open_set.push(southwest);
            closed_set.insert(southwest);
			backtrack_map[x - 1][y + 1] = current;
		}

		position west = position(x - 1, y);
		if(x >= 1 && cost_map[x - 1][y] < 200
           && closed_set.find(west) == closed_set.end())
		{
			open_set.push(west);
            closed_set.insert(west);
			backtrack_map[x - 1][y] = current;
		}

		position northwest = position(x - 1, y - 1);
		if(x >= 1 && y >= 1 && cost_map[x - 1][y - 1] < 200
           && closed_set.find(northwest) == closed_set.end())
		{
			open_set.push(northwest);
            closed_set.insert(northwest);
			backtrack_map[x - 1][y - 1] = current;
		}
    } // while

    return false;
} // make_reachable_collection

// Starts at target and backtracks, adding information to the solution path
// Then once it hits the starting position, stops
void a_star_test::backtracker(vector<position> &solution_path)
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
    solution_path.push_back(start);
} // backtracker

// Calculates Euclidean distance between two positions
double a_star_test::distance(position a, position b)
{
    double x_diff = a.x - b.x;
    double y_diff = a.y - b.y;
    return sqrt((x_diff * x_diff) + (y_diff * y_diff));
}

// When given a position, calculates the minimum possible cost of traveling from
// said position to any of the adjacent spaces
unsigned int a_star_test::min_cost(position p)
{
    unsigned int min = 0;
    unsigned int x = p.x;
    unsigned int y = p.y;

    if(x > 0 && cost_map[x - 1][y] < min)
    {
        min = cost_map[x - 1][y];
    }

    if(x + 1 < costmap_height && cost_map[x + 1][y] < min)
    {
        min = cost_map[x + 1][y];
    }

    if(y > 0 && cost_map[x][y - 1] < min)
    {
        min = cost_map[x][y - 1];
    }

    if(y + 1 < costmap_width && cost_map[x][y + 1] < min)
    {
        min = cost_map[x][y + 1];
    }

    if(x > 0 && y > 0 && cost_map[x - 1][y - 1] < min)
    {
        min = cost_map[x - 1][y - 1];
    }

    if(x + 1 < costmap_height && y + 1 < costmap_width && cost_map[x + 1][y + 1] < min)
    {
        min = cost_map[x + 1][y + 1];
    }

    if(x + 1 < costmap_height && y > 0 && cost_map[x + 1][y - 1] < min)
    {
        min = cost_map[x + 1][y - 1];
    }

    if(x > 0 && y + 1 < costmap_width && cost_map[x - 1][y + 1] < min)
    {
        min = cost_map[x - 1][y + 1] < min;
    }

    return min;
}
