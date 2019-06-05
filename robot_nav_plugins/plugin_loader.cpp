#include <pluginlib/class_loader.h>
#include <polygon_interface_package/polygon.h>

//... some code ...

pluginlib::ClassLoader<polygon_namespace::Polygon> poly_loader("polygon_interface_package", "polygon_namespace::Polygon");

try
{
boost::shared_ptr<polygon_namespace::Polygon> poly = poly_loader.createInstance("rectangle_namespace::Rectangle");

//... use the polygon, boost::shared_ptr will automatically delete memory when it goes out of scope
}

catch(pluginlib::PluginlibException& ex)
{
//handle the class failing to load
ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
}