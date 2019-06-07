#include "WhiteLineDetection.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>



int main(int argc, char ** argv)
{
  ros::init(argc,argv,"whiteLineDetect");
  ros::NodeHandle n;
  DetectWhiteLines detecter(n);
  ros::Rate loop_rate(3000);

  tf2_ros::Buffer buff;
  tf2_ros::TransformListener trans(buff);

  detecter.loadBuffer(&buff);

  //ros::Subscriber sub = n.subscribe<sensor_msgs::Imu>
   // ("/imu/data",1000, [&detecter](const sensor_msgs::ImuConstPtr &imu) {detecter.imuTransform(imu);});
  ros::Timer whiteLineTime = n.createTimer(ros::Duration(0.5), [&detecter](const ros::TimerEvent& t ){ detecter.detect(t);});
  ros::spin();
  return  0;
}
 
