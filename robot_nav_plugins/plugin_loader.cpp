#include <pluginlib/class_loader.h>
// fix this next line
#include <polygon_interface_package/polygon.h>

// ... some code ...
pluginlib::ClassLoader<nav_core_adapter::CostmapAdapter> costmap_loader("nav_core2::Costmap", "nav_core_adapter::CostmapAdapter");

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