# make_goal
make_goal ROS package


Subscribes to topic "nav_msgs" and expects a "nav_msgs/Odometry" input.
Publishes to topic "simple_goal" a "move_base_msgs/MoveBaseGoal" output.

## Compile
Compile in ROS Lunar with catkin_make_isolated. Note that the newest version of tf2_geometry_msgs is required (newer than findable on apt as of 5.13.18).
$catkin_make_isolated --instal --pkg make_goal

## Run
$rosrun make_goal make_goal [gps_coord_file.txt]
