#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//#include <node_handle.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

struct ParsedLine{
    double time;
    geometry_msgs::Twist twist;
};

ParsedLine parse_line(const std::string &line_in) {
    std::istringstream line_stream(line_in);
    ParsedLine line;
    line_stream >> line.time >> line.twist.linear.x >> line.twist.linear.y >> line.twist.linear.z >>
        line.twist.angular.x >> line.twist.angular.y >> line.twist.angular.z;
    return line;
}

void run(int argc, char** argv) {
    ros::init(argc, argv, "sim_instr");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    std::string filename;
    nh.getParam("filename", filename);
    std::ifstream file(filename);

    std::vector<geometry_msgs::Twist> twists;
    std::vector<ros::Timer> timers;

    auto callback = [&pub, &twists](auto) {
    pub.publish(twists.back());
    twists.pop_back();
    };
    while(file) {
        std::string line;
        std::getline(file, line);

        ParsedLine pline = parse_line(line);
        twists.push_back(pline.twist);
        timers.push_back(nh.createTimer(ros::Duration(pline.time), callback, true));
    }
}

int main(int argc, char** argv) {
    run(argc, argv);
}