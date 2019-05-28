##include "a_star_test.h"

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
        check_adjacent(southeast, current);
        
        // add east state
        position east = position(x + 1, y, current);
        check_adjacent(east, current);
        
        // add northeast state
        position northeast = position(x + 1, y - 1, current);
        check_adjacent(northeast, current);
        
        // add south state
        position south = position(x, y + 1, current);
        check_adjacent(south, current);
        
        // add north state
        position north = position(x, y - 1, current);
        check_adjacent(north, current);
        
        // add southwest state
        position southwest = position(x - 1, y + 1, current);
        check_adjacent(southwest, current);
        
        // add west state
        position west = position(x - 1, y, current);
        check_adjacent(west, current);
        
        // add northwest state
        position northwest = position(x - 1, y - 1, current);
        check_adjacent(northwest, current);
    } // while
    
    return false;
} // make_reachable_collection

// Check adjacent state
void a_star_test::check_adjacent(position &p, position &c)
{
    // add only if it is within the bounds of the cost_map,
    // reachable (cost_map val < 200), and it hasn't been added already
    if((p.x < costmap_height) && (p.y < costmap_width) &&
       (p.x >= 0) && (p.y >= 0)
       && cost_map[p.x][p.y] < 200
       && closed_set.find(p) == closed_set.end())
    {
        open_set.push(p);
        backtrack_map[p.x][p.y] = c;
    }
}

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
