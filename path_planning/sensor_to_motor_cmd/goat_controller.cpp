using namespace std;
#include <vector>
#include <priority_queue>
#include <cmath>

int main(int argc, char** argv)
{
    // Placeholder for the cost_map we receive as input
    // 2d vector of ints [0, 255] where 255 is considered the most unreachable
    vector<vector<unsigned int>> cost_map;

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

        // Load the values of start and target into a_star
        a_star::init(start, target);

        // Attempt to find a solution
        priority_queue<position, weight_compare> positions;
        foundtarget = a_star::make_reachable_collection();
    } // while

    vector<a_star::position> solution_path;

    // Backtrack from the target and add positions to solution path
    backtracker(solution_path);

    return 0;
}