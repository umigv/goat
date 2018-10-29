#include "cmd_vel_selector_node.h"

#include <ros/ros.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cmd_vel_selector_node");

    ros::NodeHandle node;
    ros::NodeHandle local_node{ "~" };

    umigv::cmd_vel_selector::CmdVelSelectorNode selector{ node, local_node };

    ros::spin();
}
