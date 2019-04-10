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
        position current = positions.front();
        positions.pop_front();
        
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

// Struct that contains a position, and the previous position
struct position
{
    position(x_in, y_in, x_p, y_p): x(x_in), y(y_in)
    {
    }
    
    unsigned int x;
    unsigned int y;
};
