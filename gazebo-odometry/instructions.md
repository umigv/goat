# Manual Call
Run `rosservice call gazebo/get_model_state odometry/filtered mybot` to manually get odometry information.

# Extracting Odometry Information
1. Run Gazebo as described in the Teleop Section of the goat_description repo
2. In a new terminal run `rosrun gazebo-odometry gazebo-odometry_node`
3. `robot_name` is a required parameter. The name of our robot model is `mybot`. `timeout` is an optional parameter, this is the time in milliseconds that the node will wait for a response from the `gazebo/get_model_state` service. The default is 250.0 milliseconds. Frequency is another optional parameter, this is the frequency at which gazebo will be polled. The default is 60.0 Hz.
4.