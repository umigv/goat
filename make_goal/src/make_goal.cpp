// Code refactored by Michael specifically for Greg
#include "goal_director.h"

#include <fstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <umigv_utilities/exceptions.hpp>
#include <umigv_utilities/rosparam.hpp>
#include <umigv_utilities/types.hpp>
#include <umigv_utilities/utility.hpp>

using namespace umigv::types;

using umigv::make_goal::GoalDirector;
using umigv::make_goal::GoalDirectorBuilder;

struct Parameters {
	std::string goal_id;
	std::string goals_filename;
	f64 threshold;
	ros::Rate rate = 1.0;
};

static Parameters get_parameters(ros::NodeHandle &node) {
	using namespace std::literals;

	Parameters params;

	try {
		params.goal_id =
			umigv::get_parameter_fatal<std::string>(node, "goal_id"s);
		params.goals_filename =
			umigv::get_parameter_fatal<std::string>(node, "goals_filename"s);
		params.threshold =
			umigv::get_parameter_fatal<f64>(node, "threshold");
	} catch (const umigv::ParameterNotFoundException &e) {
		ROS_FATAL_STREAM("unable to find parameter '" << e.parameter() << "'");
		umigv::blocking_shutdown();
	}

	params.rate = ros::Rate{ umigv::get_parameter_or(node, "rate"s, 1.0) };

	return params;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "make_goal");

	ros::NodeHandle node;
	ros::NodeHandle private_node{ "~" };

	Parameters params = get_parameters(private_node);

	std::ifstream ifs{ params.goals_filename };

	GoalDirector director =
		GoalDirectorBuilder{ }.with_node(node)
							  .from_stream(ifs)
							  .with_threshold(params.threshold)
							  .with_goal_id(std::move(params.goal_id))
							  .build();

	const auto subscriber = node.subscribe<sensor_msgs::NavSatFix>(
		"fix", 10, &GoalDirector::update_fix, &director
	);
	const auto timer =
		node.createTimer(params.rate, &GoalDirector::publish_goal, &director);

	ros::spin();
}
