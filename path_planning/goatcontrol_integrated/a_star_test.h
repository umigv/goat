#ifndef A_STAR_TEST_H_
#define A_STAR_TEST_H_

#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <functional>

// Struct that contains an (x, y) pair of coordinates
struct position
{
	position(): x(0), y(0) {}
	position(unsigned int x_in, unsigned int y_in): x(x_in), y(y_in) {}

	bool operator==(const position &other) const
	{
		if(other.x == this->x && other.y == this->y)
		{
			return true;
		}
		return false;
  	}
	unsigned int x;
	unsigned int y;
}; // position struct

// Struct that contains an (x, y) pair of coordinates
struct position_hasher
{
	size_t operator()(const position &pos) const
	{
		return std::hash<unsigned int>()(pos.x);
	}
}; // position struct

// Struct that contains an (x, y) pair of coordinates
struct position_comparator
{
	bool operator()(const position &pos1, const position &pos2) const
	{
		if(pos1.x == pos2.x && pos1.y == pos2.y)
		{
			return true;
		}
		return false;
	}
}; // position struct

class a_star_test
{
public:
    // Fill costmap and its width/height and starting and target positions with values for testing
    a_star_test(int width, int height, position strt, position trg,
                std::vector<std::vector<unsigned int> > costs);

    // Breadth first search 
    bool make_reachable_collection();

    // Starts at target and backtracks, adding information to the solution path
    // Then once it hits the starting position, stops and reverses the solution path vector
    void backtracker(std::vector<position> &solution_path);

    // Calculates Euclidean distance between two positions
    double distance(position a, position b);

    // When given a position, calculates the minimum possible cost of traveling from
    // said position to any of the adjacent spaces
    unsigned int min_cost(position p);

    // void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    struct weight_compare
    {
        a_star_test* gc;

        // pathfinding heuristic as written and described in [insert stanford article source here]
        bool operator()(position a, position b)
        {
            /*
            double d = std::min(gc->min_cost(a), gc->min_cost(b));
            if(d == 0 && (gc->cost_map[a.x][a.y] == gc->cost_map[b.x][b.y]))
            {
                d = 1;
            } */
            const double weight_a = gc->cost_map[a.x][a.y];
            const double weight_b = gc->cost_map[b.x][b.y];
            
            /*
             const double weight_a = (0.2 * gc->distance(a, gc->target)) + gc->cost_map[a.x][a.y];
             const double weight_b = (0.2 * gc->distance(b, gc->target)) + gc->cost_map[b.x][b.y];
             */
            return weight_a < weight_b;
        }
    }; // comparator for priority queue

    friend struct position;
    friend class weight_compare;

protected:

    // create a position object that holds the starting position
    position start;

    // create a position object that holds the position of the target
    position target;

    // priority queue to order current positions and eventually find the target
    std::priority_queue<position, std::vector<position>, weight_compare> open_set{weight_compare{this}};

    // unordered set to store the positions that have already been visited
    std::unordered_set<position, position_hasher, position_comparator> closed_set;

    // cost map from input that stores the values of the costs of each location
    std::vector<std::vector<unsigned int> > cost_map;

    // data structure for backtracking
    std::vector<std::vector<position> > backtrack_map;

    int costmap_width;
    int costmap_height;
};
#endif
