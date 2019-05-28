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
        // cout << "Current position: " << current.x << " " << current.y << '\n';
        open_set.pop();

        // if it is the target, exit the loop
        if(current == target)
        {
            return true;
        } // if

        if(closed_set.find(current) != closed_set.end()){
            continue; // skip this iteration of the loop
        }

        closed_set.insert(current);

        // else add all adjacent states to the queue
		unsigned int x = current.x;
		unsigned int y = current.y;

		// add southeast state
    	position southeast = position(x + 1, y + 1, current);
    	// but only if it is within the bounds of the cost_map,
    	// reachable (cost_map val < 200), and it hasn't been added already
	    if((x + 1 < costmap_height) && (y + 1 < costmap_width)
           && cost_map[x + 1][y + 1] < 200
           && closed_set.find(southeast) == closed_set.end())
	    {
			open_set.push(southeast);
			backtrack_map[x + 1][y + 1] = current;
	    }

		position east = position(x + 1, y, current);
		if((x + 1 < costmap_height) && cost_map[x + 1][y] < 200
            && closed_set.find(east) == closed_set.end())
		{
			open_set.push(east);
			backtrack_map[x + 1][y] = current;
		}

		position northeast = position(x + 1, y - 1, current);
		if((x + 1 < costmap_height) && y >= 1 && cost_map[x + 1][y - 1]  < 200
            && closed_set.find(northeast) == closed_set.end())
		{
			open_set.push(northeast);
			backtrack_map[x + 1][y - 1] = current;
		}

		position south = position(x, y + 1, current);
		if((y + 1 < costmap_width) && cost_map[x][y + 1] < 200
            && closed_set.find(south) == closed_set.end())
		{
			open_set.push(south);
			backtrack_map[x][y + 1] = current;
		}

		position north = position(x, y - 1, current);
		if(y >= 1 && cost_map[x][y - 1] < 200
            && closed_set.find(north) == closed_set.end())
		{
			open_set.push(north);
			backtrack_map[x][y - 1] = current;
		}

		position southwest = position(x - 1, y + 1, current);
		if(x >= 1 && (y + 1 < costmap_width) && cost_map[x - 1][y + 1] < 200
            && closed_set.find(southwest) == closed_set.end())
		{
			open_set.push(southwest);
			backtrack_map[x - 1][y + 1] = current;
		}

		position west = position(x - 1, y, current);
		if(x >= 1 && cost_map[x - 1][y] < 200
            && closed_set.find(west) == closed_set.end())
		{
			open_set.push(west);
			backtrack_map[x - 1][y] = current;
		}

		position northwest = position(x - 1, y - 1, current);
		if(x >= 1 && y >= 1 && cost_map[x - 1][y - 1] < 200
            && closed_set.find(northwest) == closed_set.end())
		{
			open_set.push(northwest);
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

