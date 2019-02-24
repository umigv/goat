#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

struct TimeAndCmdVel{
    ros::Duration time;
    geometry_msgs::Twist cmd_vel;
};

std::istream& operator >> (std::istream& is, TimeAndCmdVel& line)
{
    std::string unparsedLine;
    if(!getline(is, unparsedLine)) {
        return is;
    }

    std::istringstream toParse(unparsedLine);

    double timeOffset;

    if (toParse >> timeOffset >> line.cmd_vel.linear.x >> line.cmd_vel.linear.y >> line.cmd_vel.linear.z >>
        line.cmd_vel.angular.x >> line.cmd_vel.angular.y >> line.cmd_vel.angular.z) {
        line.time = ros::Duration(timeOffset);

    }
    else {
        is.setstate(std::ios_base::failbit);
    }

    return is;
}


int main(int argc, char** argv) { 
    ros::init(argc, argv, "sim_instr");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	while (pub.getNumSubscribers() == 0) {
		ros::spinOnce();
	}

    std::string filename;
    if (!nh.getParam("filename", filename)) {
        ROS_FATAL_STREAM("Missing Parameter: filename");
        ros::shutdown();
		ros::waitForShutdown();
		return 1;
    }

    std::ifstream file(filename);

    if (!file.is_open()) {
        ROS_FATAL_STREAM("Invalid Filename" << filename);
        return 1;
	}

	std::vector<TimeAndCmdVel> lines(std::istream_iterator<TimeAndCmdVel>(file), {});
	std::vector<ros::Timer> timers;
	timers.reserve(lines.size());


	while (!ros::Time::isValid()) {
		ros::spinOnce();
		
	}

	std::transform(lines.cbegin(), lines.cend(), std::back_inserter(timers), [&nh, &pub](const TimeAndCmdVel &directive) {

		auto callback = [cmd_vel = directive.cmd_vel, &pub](const ros::TimerEvent& event) {
			pub.publish(cmd_vel);			
		};
		return nh.createTimer(directive.time, callback, true);
	});

	auto comp = [](const TimeAndCmdVel &lhs, const TimeAndCmdVel &rhs) { return lhs.time < rhs.time; };

	if(!std::is_sorted(timers.begin(), timers.end()), comp) {

		ROS_WARN_STREAM("Commands in the Input file should be in time-increasing order, resolved internally");
	}
	ros::spin();
}
