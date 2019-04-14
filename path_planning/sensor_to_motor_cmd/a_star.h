#include <vector>
#include <priority_queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>

class a_star
{
public:
    // Struct that contains an (x, y) pair of coordinates
    struct position {}; // position struct

    a_star(position str, position tar, vector<vector<unsigned int>> &c_m):
    start(str), target(tar), cost_map(c_m);
    {
        // Add the starting position to the priority queue to begin
        open_set.push(start);

        // Resize the backtrack_map to the size of the costmap
        backtrack_map.resize(cost_map.size(), vector<position>(cost_map[0].size()));
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

    // When given a position, calculates the minimum possible cost of traveling from
    // said position to any of the adjacent spaces
    unsigned int min_cost(position p); // min_cost

    // Comparator to take into account cost and distance of two positions
    class weight_compare {}; // comparator for priority queue

private:
    // create a position object that holds the starting position
    position start;

    // create a position object that holds the position of the target
    position target;

    // priority queue to order current positions and eventually find the target
    priority_queue<position, weight_compare> open_set;

    // unordered set to store the positions that have already been visited
    unordered_set<position> closed_set;

    // cost map from input that stores the values of the costs of each location
    vector<vector<unsigned int>> cost_map;

    // data structure for backtracking
    vector<vector<position>> backtrack_map;

    friend struct position;
    friend class weight_compare;
};