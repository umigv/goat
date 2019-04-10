class a_star
{
public:
    a_star(): start(position(0, 0)), target(position(0, 0))
    {
    }

    // Update the current target
    void set_target();

    // Breadth first search 
    bool make_reachable_collection(); // make_reachable_collection

    // Returns whether the position is reachable or not
    void add_reachable_adjacent(unsigned int x, unsigned int y); // add_reachable_adjacent

    // Starts at target and backtracks, adding information to the solution path
    // Then once it hits the starting position, stops and reverses the solution path vector
    void backtracker(vector<position> &solution_path); // backtracker

    // Calculates Euclidean distance between two positions
    double distance(position a, position b); // distance

    // Struct that contains an (x, y) pair of coordinates
    struct position {}; // position struct

    // Comparator to take into account cost and distance of two positions
    class weight_compare {}; // comparator for priority queue

private:
    // create a position object that holds the starting position
    position start;

    // create a position object that holds the position of the target
    position target;

    // priority queue to order current positions and eventually find the target
    priority_queue<position, weight_compare> positions;

    friend struct position;
    friend class weight_compare;
};