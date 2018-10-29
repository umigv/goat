## Synopsis

`umigv_utilities` contains a few header-only libraries that UMIGV has found we use regularly enough to put into a package.

## Code Example

    #include <umigv_utilities/rosparam.hpp>

    ros::NodeHandle private_handle{ "~" };
    auto frame_id = umigv::get_parameter_fatal<std::string>(private_handle, "frame_id");

## Motivation

`umigv_utilities` was created as a convenience factor for us to place all of our commonly used code sections into one package, so we can make changes to the implementation and add new functionality as desired.

## Installation

Clone this repository into your `catkin` workspace, then run `catkin_make`. Optionally, run `catkin_make install`.

## Contributors

`umigv_utilities` is authored and maintained by Gregory Meyer.
