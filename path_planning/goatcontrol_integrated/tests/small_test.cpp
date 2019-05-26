#include <catch2/catch.hpp>

TEST_CASE("comparator_everything_equal", "[comparator][group2]") {
    position target{1, 4};
    position start{4, 0};
    
    std::vector<std::vector<unsigned int> > costs(5, std::vector<unsigned int>(5, 0));
    
    costs[0][0] = 65;
    costs[1][0] = 34;
    costs[2][0] = 50;
    costs[3][0] = 230;
    costs[4][1] = 255;
    costs[4][2] = 210;
    costs[1][2] = 210;
    costs[2][2] = 255;
    costs[3][2] = 150;
    costs[3][3] = 90;
    costs[1][3] = 205;
    costs[2][3] = 255;
    costs[2][4] = 50;
    costs[3][4] = 80;
    
    a_star_test a(5, 5, start, target, costs);
    
    // Run A* algorithm by adding unvisited adjacent states to queue,
    // sorting them by heuristic, until the target is found
    a.make_reachable_collection();
    
    vector<position> solution_path;
    
    // Backtrack from the target and add positions to solution path
    // Solution path is given in reverse order, starting from the goal
    // and listing the previous position until it reaches the start
    a.backtracker(solution_path);
    
    // Reverse the solution path so that it goes from the start node to the target
    reverse(solution_path.begin(), solution_path.end());
}