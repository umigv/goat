using namespace std;
#include <queue>

int main(int argc, char** argv)
{
    queue<position> positions;
}

void make_reachable_collection()
{
    while(!position.empty())
    {
        position current = states.front();
        position.pop_front();
        
        add_reachable_adjacent(current.x, current.y);
    }
}

// Returns whether the position is reachable or not
void add_reachable_adjacent(unsigned int x, unsigned int y)
{
    // If the position contains an obstacle or is very close to an obstacle
    // mark it as not reachable, for all adjacent states
    
    if(cost_map(x + 1, y + 1) < 200)
    {
        positions.push(position(x + 1, y + 1, x, y));
    }
    
    if(cost_map(x + 1, y) < 200)
    {
        positions.push(position(x + 1, y, x, y));
    }
    
    if(cost_map(x + 1, y - 1)  < 200)
    {
        positions.push(position(x + 1, y - 1, x, y));
    }
    
    if(cost_map(x, y + 1)  < 200)
    {
        positions.push(position(x, y + 1, x, y));
    }
    
    if(cost_map(x, y - 1)  < 200)
    {
        positions.push(position(x, y - 1, x, y));
    }
    
    if(cost_map(x - 1, y + 1)  < 200)
    {
        positions.push(position(x - 1, y + 1, x, y));
    }
    
    if(cost_map(x - 1, y)  < 200)
    {
        positions.push(position(x - 1, y, x, y));
    }
    
    if(cost_map(x - 1, y - 1)  < 200)
    {
        positions.push(position(x - 1, y - 1, x, y));
    }
}

// Struct that contains a position, and the previous position
struct position
{
    position(x_in, y_in, x_p, y_p): x(x_in), y(y_in), prev_x(x_p), prev_y(y_p)
    {
    }
    
    unsigned int x;
    unsigned int y;
    
    unsigned int prev_x;
    unsigned int prev_y;
};
