#include <ros/ros.h>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n("~");

  //param: vector<double> lat, vector<double> long
  vector<double> latitude;
  vector<double> longitude;

  bool receivedLat = n.getParam("latitude", latitude);
  bool receivedLong = n.getParam("longitude", longitude);
  const double altitude = n.param("altitude", 0.0);

  if(!receivedLat || !receivedLong)
  {
    ROS_FATAL("Did not receive either latitude or longitude as a parameter");
    return EXIT_FAILURE;
  }

  if(latitude.size() != longitude.size())
  {
    ROS_FATAL("Latitude and longitude not the same length.");
    return EXIT_FAILURE;
  }

  vector<geographic_msgs::GeoPoint> geo_points;

  transform(latitude.begin(), latitude.end(), longitude.begin(), back_inserter(geo_points), [](double lat, double lon){return geodesy::toMsg(lat, lon);});

  ros::NodeHandle global_nh;
  ros::Publisher pub = global_nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

  while(pub.getNumSubscribers() == 0)
  {
    ros::spinOnce();
  }

  geodesy::UTMPoint utm;
  uint32_t seq = 0;

  for(const geographic_msgs::GeoPoint& point : geo_points)
  {
    geodesy::fromMsg(point, utm);
    geometry_msgs::PoseStamped to_publish;
    to_publish.header.seq = seq++;
    to_publish.header.stamp = ros::Time::now();
    to_publish.header.frame_id = "utm";
    to_publish.pose.position.x = utm.easting;
    to_publish.pose.position.y = utm.northing;
    to_publish.pose.position.z = altitude;
    to_publish.pose.orientation.x = 1;
    to_publish.pose.orientation.y = 0;
    to_publish.pose.orientation.z = 0;
    to_publish.pose.orientation.w = 0;

    pub.publish(to_publish);
  }

  ros::spin();
}
