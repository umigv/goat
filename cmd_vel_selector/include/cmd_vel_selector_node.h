#ifndef UMIGV_CMD_VEL_SELECTOR_CMD_VEL_SELECTOR_NODE_H
#define UMIGV_CMD_VEL_SELECTOR_CMD_VEL_SELECTOR_NODE_H

#include <type_traits>
#include <utility>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

namespace umigv {
namespace cmd_vel_selector {

class CmdVelSelectorNode;

class Publishers {
public:
    explicit Publishers(ros::NodeHandle &node);

    void publish_cmd_vel(const geometry_msgs::Twist &cmd_vel);

    void publish_source(const std_msgs::String &cmd_vel_source);

private:
    ros::Publisher cmd_vel_;
    ros::Publisher source_;
};

class Subscribers {
public:
    Subscribers(ros::NodeHandle &node, CmdVelSelectorNode &selector);

private:
    ros::Subscriber teleop_;
    ros::Subscriber autonomous_;
    ros::Subscriber joy_;
};

class Parameters {
public:
    explicit Parameters(ros::NodeHandle &node);

    int joy_button() const noexcept;

    int pressed_threshold() const noexcept;

private:
    int joy_button_;
    int pressed_threshold_;
};

class CmdVelSelectorNode {
public:
    CmdVelSelectorNode(ros::NodeHandle node, ros::NodeHandle local_node);

    void accept_teleop(const geometry_msgs::Twist::ConstPtr &cmd_vel_ptr);

    void accept_autonomous(const geometry_msgs::Twist::ConstPtr &cmd_vel_ptr);

    void check_button(const sensor_msgs::Joy::ConstPtr &joy_ptr);

private:
    void toggle_if_pressed(int data);

    ros::NodeHandle node_;
    ros::NodeHandle local_node_;
    Subscribers subs_{ node_, *this };
    Publishers pubs_{ node_ };
    Parameters params_{ local_node_ };
    bool is_pressed_ = false;
    bool is_teleop_ = true;
};

} // namespace cmd_vel_selector
} // namespace umigv

#endif
