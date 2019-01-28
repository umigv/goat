#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>


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

    std::string filename;
    if (!nh.getParam("filename", filename)) {
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

    auto callback = [&pub, &lines](auto) {
    pub.publish(lines.back().cmd_vel);
    lines.pop_back();
    };

    std::transform(lines.cbegin(), lines.cend(), std::back_inserter(timers), [&nh, &callback](const TimeAndCmdVel &directive) { 
        return nh.createTimer(directive.time, callback, true);
    });

    ros::spin();
}
