#include "cmd_vel_selector_node.h"

#include <cstdint>
#include <limits>
#include <utility>

namespace umigv {
namespace cmd_vel_selector {

constexpr char TELEOP_STRING[] = "teleoperated";
constexpr char AUTONOMOUS_STRING[] = "autonomous";

Publishers::Publishers(ros::NodeHandle &node)
: cmd_vel_{ node.advertise<geometry_msgs::Twist>("cmd_vel", 10) },
  source_{ node.advertise<std_msgs::String>("cmd_vel_source", 10, true) } {
    std_msgs::String to_publish;

    to_publish.data = TELEOP_STRING;

    publish_source(to_publish);
}

void Publishers::publish_cmd_vel(const geometry_msgs::Twist &cmd_vel) {
    cmd_vel_.publish(cmd_vel);
}

void Publishers::publish_source(const std_msgs::String &cmd_vel_source) {
    source_.publish(cmd_vel_source);
}

Subscribers::Subscribers(ros::NodeHandle &node, CmdVelSelectorNode &selector)
: teleop_{
      node.subscribe<geometry_msgs::Twist>("teleop/cmd_vel", 10,
                                           &CmdVelSelectorNode::accept_teleop,
                                           &selector)
  }, autonomous_{ node.subscribe<geometry_msgs::Twist>(
      "autonomous/cmd_vel", 10,
      &CmdVelSelectorNode::accept_autonomous, &selector
  ) }, joy_{
      node.subscribe<sensor_msgs::Joy>("joy", 10,
                                       &CmdVelSelectorNode::check_button,
                                       &selector)
  } { }

Parameters::Parameters(ros::NodeHandle &node) {
    node.param("joy_button", joy_button_, 0);
    node.param("pressed_threshold", pressed_threshold_, 1);

    if (joy_button_ < 0) {
        ROS_WARN_STREAM("invalid ~joy_button: " << joy_button_);
        joy_button_ = 0;
    }

    if (pressed_threshold_ <= 0) {
        ROS_WARN_STREAM("invalid ~pressed_threshold: " << pressed_threshold_);
        pressed_threshold_ = 1;
    }
}

int Parameters::joy_button() const noexcept {
    return joy_button_;
}

int Parameters::pressed_threshold() const noexcept {
    return pressed_threshold_;
}

CmdVelSelectorNode::CmdVelSelectorNode(ros::NodeHandle node,
                                       ros::NodeHandle local_node)
: node_{ std::move(node) }, local_node_{ std::move(local_node) } { }

void CmdVelSelectorNode::accept_teleop(
    const geometry_msgs::Twist::ConstPtr &cmd_vel_ptr
) {
    ROS_DEBUG_STREAM("received a teleoperated velocity command");

    if (is_teleop_) {
        pubs_.publish_cmd_vel(*cmd_vel_ptr);
        ROS_DEBUG_STREAM("published a teleoperated velocity command");
    }
}

void CmdVelSelectorNode::accept_autonomous(
    const geometry_msgs::Twist::ConstPtr &cmd_vel_ptr
) {
    ROS_DEBUG_STREAM("received an autonomous velocity command");

    if (!is_teleop_) {
        pubs_.publish_cmd_vel(*cmd_vel_ptr);
        ROS_DEBUG_STREAM("published an autonomous velocity command");
    }
}

void CmdVelSelectorNode::check_button(
    const sensor_msgs::Joy::ConstPtr &joy_ptr
) {
    if (params_.joy_button() >= joy_ptr->buttons.size()) {
        ROS_ERROR_STREAM(
            "CmdVelSelectorNode::check_button: invalid button "
            << params_.joy_button()
        );

        return;
    }

    const auto data = static_cast<int>(joy_ptr->buttons[params_.joy_button()]);

    ROS_DEBUG_STREAM("received a sensor_msgs/Joy message with button data "
                     << data);

    toggle_if_pressed(data);
}

void CmdVelSelectorNode::toggle_if_pressed(const int data) {
    const bool new_state = data >= params_.pressed_threshold();

    if (new_state && !is_pressed_) {
        is_teleop_ = !is_teleop_;

        const std_msgs::String to_publish = [this] {
            std_msgs::String string;

            if (is_teleop_) {
                string.data = TELEOP_STRING;
            } else {
                string.data = AUTONOMOUS_STRING;
            }

            return string;
        }();

        if (is_teleop_) {
            ROS_INFO_STREAM("switching to teleoperated mode");
        } else {
            ROS_INFO_STREAM("switching to autonomous mode");
        }

        pubs_.publish_source(to_publish);
    }

    is_pressed_ = new_state;
}

} // namespace cmd_vel_selector
} // namespace umigv
