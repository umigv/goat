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

/*bool compareByLength(const  TimeAndCmdVel &t1, const TimeAndCmdVel &t2) {
    		return t1.time > t2.time;
}*/

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
    if (!nh.getParam("param", filename)) {
        ROS_FATAL_STREAM("Missing Parameter: Filename");
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

	bool lines_sorted;
	for(int i = 0; i < (lines.size() - 1); ++i) {
		if(!(lines.at(i + 1).time > lines.at(i).time)) {
			ROS_WARN_STREAM("Commands in the Input file should be in time-increasing order, resolved internally");
			lines_sorted = false;
			break;
		}
	}


	while (!ros::Time::isValid()) {
		std::cout << "spin once" << std::endl;
		ros::spinOnce();
		
	}

	for (const TimeAndCmdVel &directive : lines) {
		timers.push_back(nh.createTimer(directive.time, [cmd_vel = directive.cmd_vel, &pub](const ros::TimerEvent&) {
			pub.publish(cmd_vel);			
		}, true));
	}

    ros::spin();
}
