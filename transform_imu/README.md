## Synopsis

`transform_imu` tranforms Imu and MagneticField messages from a NED world frame to an ENU world frame. Or the other way around, since the transform is identical the other way.

## Example

    roslaunch goat_launch imu.launch

## Motivation

`transform_imu` was created out of necessity for compatibility with `robot_localization`. The PhidgetSpatial series of IMUs output data in the NED world frame, as seen on their casing, but `robot_localization` requires an ENU world frame to correctly work with sensors. Without an off-the-shelf transforming solution, we had to come up with one of our own.

## Installation

Clone this repository into your `catkin` workspace, then run `catkin_make_isolated --install` and then `source install_isolated/setup.bash` to get access to it.

## Contributors

`transform_imu` is authored and maintained by Gregory Meyer.
