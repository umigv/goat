using namespace std;
#include <vector>

int main(int argc, char** argv)
{
    // Placeholder for the cost_map we receive as input
    // 2d vector of ints [0, 255] where 255 is considered the most unreachable
    vector<vector<unsigned int>> cost_map_in;

    // Placeholders for the coordinates of the starting position
    unsigned int start_x = 0;
    unsigned int start_y = 0;

    // Placeholders for gps coordinates for the target
    unsigned int gps_target_x = 0;
    unsigned int gps_target_y = 0;

    bool foundtarget = false;

    // Loop while a solution isn't found
    while(!foundtarget)
    {
        // Create a position object that holds the starting position
        a_star::position start(start_x, start_y);

        // Create a position object that holds the position of the target
        a_star::position target(gps_target_x, gps_target_y);

        // Make a new a_star object
        a_star pathfinder = a_star(vector<vector<unsigned int>> cost_map_in);

        // Attempt to find a solution
        foundtarget = pathfinder.make_reachable_collection();
    } // while

    vector<a_star::position> solution_path;

    // Backtrack from the target and add positions to solution path
    // Solution path is given in reverse order, starting from the goal 
    // and listing the previous position until it reaches the start
    pathfinder.backtracker(solution_path);

    return 0;
}