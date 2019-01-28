#include "WhiteLineDetection.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <geometry_msgs/Quaternion>


int main(int argc, char ** argv)
{
  
  ros::init(argc,argv,"whiteLineDetect");
  // ros::init(argc,argv,"test");
  ros::NodeHandle n;
  DetectWhiteLines detecter(n);
  ros::Rate loop_rate(3000);
  ros::Subscriber sub = n.subscribe<std_msgs::String>
    ("chatter",1000, [&detecter](const std_msgs::String::ConstPtr &str) { detecter.imuTransform(str); });
  ros::Timer tim = n.createTimer(ros::Duration(0.1), [&detecter](const ros::TimerEvent& t ){ detecter.detect(t);});
  ros::spin();
return  0;
}
 
