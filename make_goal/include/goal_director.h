#ifndef UMIGV_MAKE_GOAL_GOAL_DIRECTOR_H
#define UMIGV_MAKE_GOAL_GOAL_DIRECTOR_H

#include "message_utils.h"

#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include <geodesy/utm.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <sensor_msgs/NavSatFix.h>
#include <umigv_utilities/types.hpp>

namespace umigv {
namespace make_goal {

using GoalT = move_base_msgs::MoveBaseActionGoal;
using WaypointT = geodesy::UTMPoint;

class GoalDirector;

class GoalDirectorBuilder {
public:
    GoalDirectorBuilder& with_node(ros::NodeHandle &node) noexcept;

    GoalDirectorBuilder& from_stream(std::istream &is) noexcept;

    GoalDirectorBuilder& with_threshold(f64 threshold) noexcept;

    GoalDirectorBuilder& with_goal_id(std::string id) noexcept;

    // must be called on an rvalue
    // throws std::logic_error if not fully initialized
    GoalDirector build();

private:
    ros::NodeHandle *node_ = nullptr;
    boost::optional<std::vector<WaypointT>> waypoints_ = boost::none;
    boost::optional<f64> threshold_ = boost::none;
    boost::optional<std::string> id_ = boost::none;
};

class GoalDirector {
public:
    friend GoalDirectorBuilder;

    GoalDirector(GoalDirector &&other) noexcept;

    void update_fix(const sensor_msgs::NavSatFix::ConstPtr &fix_ptr);

    void publish_goal(const ros::TimerEvent&);

private:
    GoalDirector(ros::Publisher publisher, std::vector<WaypointT> waypoints,
                 f64 threshold, std::string id) noexcept;

    bool is_goal_reached(const sensor_msgs::NavSatFix &fix) const noexcept;

    boost::optional<std::reference_wrapper<const WaypointT>>
    current() const noexcept;

    boost::optional<std::size_t> current_index() const noexcept;

    ros::Publisher publisher_;
    std::vector<WaypointT> waypoints_;
    f64 distance_threshold_;
    std::vector<WaypointT>::const_iterator current_iter_ = waypoints_.cbegin();
    std::uint32_t seq_ = 0;
    std::string id_;
};

} // namespace make_goal
} // namespace umigv

#endif
