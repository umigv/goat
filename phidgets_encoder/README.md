## Synopsis

`phidgets_encoder` interfaces with a Phidgets 1047 High Speed Encoder board and outputs a JointState for two wheels, including instantaneous rotational velocity.

## Code Example

	rosrun phidgets_encoder phidgets_encoder_node _rads_per_tick:=0.00436332313 _serial_number:=-1 _frequency:=60.0 _frame_id:=encoders

## Motivation

Getting a JointState for each encoder-bound wheel is the first step to generating Odometry data from encoder data. Combining the JointState messages with data from a URDF will allow us to broadcast Odometry messages based on encoder data.

## Installation

Clone this repository into your `catkin` workspace, then run `catkin_make_isolated --install`.

## Contributors

`phidgets_encoder` is authored and maintained by Gregory Meyer of UMIGV.
