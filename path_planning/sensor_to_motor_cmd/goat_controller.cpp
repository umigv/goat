using namespace std;
#include <priority_queue>

// TO DO: comparator that takes into account custom heuristic to order the priority queue

// TO DO: data structure for backtracking so that after we find the target,
// we can add the path we took to our final solution vector

int main(int argc, char** argv)
{
    // placeholder for the cost_map we receive as input
    // 2d vector of ints [0, 255] where 255 is considered the most unreachable
    vector<vector<unsigned int>> cost_map;

    // placeholders for gps coordinates for the target
    int gps_target_x = -1;
    int gps_target_y = -1;

    // create a position object that holds the position of the target
    position target(gps_target_x, gps_target_y);

    bool foundtarget = false;
    priority_queue<position> positions;
    vector<position> solution_path;
}

// Breadth first search 
void make_reachable_collection()
{
    while(!positions.empty())
    {
        position current = positions.front();
        positions.pop_front();
        
        // if it is the target, exit the loop
        if(current == target)
        {
            foundtarget = true;
            break;
        } // if

        // else add all adjacent states to the queue
        add_reachable_adjacent(current.x, current.y);
    }
}

// Returns whether the position is reachable or not
void add_reachable_adjacent(unsigned int x, unsigned int y)
{
    // If the position contains an obstacle or is very close to an obstacle
    // mark it as not reachable, for all adjacent states
    // TO DO: add backtracking positions to backtracking data structure in every if statement
    
    if(cost_map(x + 1, y + 1) < 200)
    {
        positions.push(position(x + 1, y + 1));
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
}

// Struct that contains a position
struct position
{
    position(x_in, y_in, x_p, y_p): x(x_in), y(y_in)
    {
    }
    
    unsigned int x;
    unsigned int y;
};
