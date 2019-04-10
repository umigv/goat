// Class for a_star algorithm
// Takes in information from sensor readings
// Outputs a vector of coordinate pairs outlining a path from the start position to the target position

// Update the current target
void a_star::init(position str, position tar)
{
    start = str;
    target = tar;
} // set_target

// Breadth first search 
bool a_star::make_reachable_collection()
{
    // Add the starting position to the priority queue to begin
    positions.push(start);

    while(!positions.empty())
    {
        position current = positions.front();
        positions.pop_front();
        
        // if it is the target, exit the loop
        if(current == target)
        {
            return true;
        } // if

        // else add all adjacent states to the queue
        add_reachable_adjacent(current.x, current.y);
    } // while

    return false;
} // make_reachable_collection

// Returns whether the position is reachable or not
void a_star::add_reachable_adjacent(unsigned int x, unsigned int y)
{
    // If the position contains an obstacle or is very close to an obstacle
    // mark it as not reachable, for all adjacent states
    // TO DO: add backtracking positions to backtracking data structure in every if statement
    
    if(cost_map(x + 1, y + 1) < 200)
    {
        positions.push(position(x + 1, y + 1));
        // add to backtracking data structure here
    }
    
    if(cost_map(x + 1, y) < 200)
    {
        positions.push(position(x + 1, y));
    }
    
    if(cost_map(x + 1, y - 1)  < 200)
    {
        positions.push(position(x + 1, y - 1));
    }
    
    if(cost_map(x, y + 1)  < 200)
    {
        positions.push(position(x, y + 1));
    }
    
    if(cost_map(x, y - 1)  < 200)
    {
        positions.push(position(x, y - 1));
    }
    
    if(cost_map(x - 1, y + 1)  < 200)
    {
        positions.push(position(x - 1, y + 1));
    }
    
    if(cost_map(x - 1, y)  < 200)
    {
        positions.push(position(x - 1, y));
    }
    
    if(cost_map(x - 1, y - 1)  < 200)
    {
        positions.push(position(x - 1, y - 1));
    }
} // add_reachable_adjacent

// Starts at target and backtracks, adding information to the solution path
// Then once it hits the starting position, stops and reverses the solution path vector
void a_star::backtracker(vector<position> &solution_path)
{

} // backtracker

// Calculates Euclidean distance between two positions
double a_star::distance(position a, position b)
{
    double x_diff = a.x - b.x;
    double y_diff = a.y - b.y;
    return sqrt((x_diff * x_diff) + (y_diff * y_diff));
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
        // TO DO: calculate relative weights of positions according to chosen heuristic
        // and return true if weight(a) < weight(b) and false otherwise
    }
}; // comparator for priority queue