#include "a_star_test.h"
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

using namespace std;

std::vector<std::vector<unsigned int> > parse_csv(std::istream &is) {
    std::string line;
    std::string number;
    std::vector<std::vector<unsigned int> > parsed;
    
    while (std::getline(is, line)) {
        parsed.emplace_back();
        std::vector<unsigned int> &this_line = parsed.back();
        
        std::istringstream iss(line);
        while (std::getline(iss, number, ',')) {
            this_line.push_back(static_cast<unsigned int>(std::stoll(number)));
        }
    }
    
    return parsed;
}

int main(int argc, char **argv)
{
    position target{483, 453};
    position start{220, 0};
    
    ifstream file(argv[1]);
    if(!file.is_open())
    {
        cout << "Error opening file: " << argv[1] << endl;
        return -1;
    }
    
    a_star_test a(512, 512, start, target, parse_csv(file));
    
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
    
    ofstream outFile(argv[2]);
    for(auto it = solution_path.begin(); it != solution_path.end(); it++)
    {
        outFile << it->x << "," << it->y << "\n";
    }
    
    return 0;
}
