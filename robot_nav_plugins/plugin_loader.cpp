#include "ros/ros.h"
#include "std_msgs/String.h"

// fix this next line (incorrect file to include for subscribing to CostMap messages)
#include cost_map_msgs/CostMap.msg

#include <pluginlib/class_loader.h>

// fix this next line
#include <polygon_interface_package/polygon.h>

#include <sstream>

int main(int argc, char **argv)
{
	// initialize ros stuff
	ros::init(argc, argv, "costmap_subscriber");
	ros::NodeHandle n;

	// load CostmapAdapter plugin
	pluginlib::ClassLoader<nav_core_adapter::CostmapAdapter>costmap_loader("nav_core2::Costmap", "nav_core_adapter::CostmapAdapter");

	try
	{
		boost::shared_ptr<nav_core_adapter::CostmapAdapter> costmap = costmap_loader.createInstance("nav_core2::Costmap");
		// ... use the costmap, boost::shared_ptr will automatically delete memory when it goes out of scope
	}

	catch(pluginlib::PluginlibException& ex)
	{
		// handle the class failing to load
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}
	
	// example of ros subscriber
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::spin();

	return 0;
}

void chatterCallback(const cost_map_msgs::CostMap::ConstPtr& msg)
{
	// Preferably, load the info from the cost map message into the costmap adapter plugin here
}