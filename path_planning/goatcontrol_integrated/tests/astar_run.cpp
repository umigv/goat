#include "goatcontrol.h"

using namespace std;

class A_star_test: public GoatControl
{
public:

	// Fill costmap with values for testing
	void fill_costmap()
	{

	}


}

int main(int argc, char **argv)
{
	A_star_test a;

	bool foundtarget = false;

	position target{1, 4};
	position start{4, 0};
	
	foundtarget = a.make_reachable_collection();

	vector<position> solution_path;

	// Backtrack from the target and add positions to solution path
	// Solution path is given in reverse order, starting from the goal 
	// and listing the previous position until it reaches the start
	listener.backtracker(solution_path);

	// Reverse the solution path so that it goes from the start node to the target
	reverse(solution_path.begin(), solution_path.end());

	return 0;
}