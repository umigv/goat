#include "encoder_state_publisher.h" // umigv::EncoderStatePublisher

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <umigv_utilities/types.hpp>
#include <umigv_utilities/utility.hpp> // umigv::blocking_shutdown
#include <umigv_utilities/exceptions.hpp> // umigv::ParameterNotFoundException,
										  // umigv::PhidgetException
#include <umigv_utilities/rosparam.hpp> // umigv:get_parameter_or,
										// umigv::get_parameter_fatal

#include <string> // operator""s
#include <tuple> // std::tuple

using namespace umigv::types;

std::tuple<int, f64, std::string>
get_defaultable_parameters(ros::NodeHandle &handle) {
	using namespace std::literals;

	return std::tuple<int, f64, std::string>{
		umigv::get_parameter_or(handle, "serial_number", -1),
		umigv::get_parameter_or(handle, "frequency", 100.0),
		umigv::get_parameter_or(handle, "frame_id", "encoders"s)
	};
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "phidgets_encoder_node");
	ros::NodeHandle handle;
	ros::NodeHandle private_handle{ "~" };

	int serial_number;
	f64 frequency;
	std::string frame_id;

	std::tie(serial_number, frequency, frame_id) =
		get_defaultable_parameters(private_handle);

	f64 rads_per_tick;

	try {
		rads_per_tick = umigv::get_parameter_fatal<f64>(private_handle,
														"rads_per_tick");
	} catch (const umigv::ParameterNotFoundException &e) {
		ROS_FATAL_STREAM("parameter '" << e.parameter() << "' not found");

		umigv::blocking_shutdown();
	}

	try {
		umigv::EncoderStatePublisher state_publisher{
			handle.advertise<sensor_msgs::JointState>("wheel_state", 10),
			umigv::RobotCharacteristics().with_frame(std::move(frame_id))
										 .with_rads_per_tick(rads_per_tick)
										 .with_serial_number(serial_number)
		};

		auto publish_timer =
			handle.createTimer(ros::Rate(frequency),
			                   &umigv::EncoderStatePublisher::publish_state,
			                   &state_publisher);

		ros::spin();
	} catch (const umigv::PhidgetsException &e) {
		ROS_FATAL_STREAM(e.what() << ": " << e.error_description() << " ("
						 << e.error_code() << ")");

		umigv::blocking_shutdown();
	}
}
