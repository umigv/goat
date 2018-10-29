#include "goal_director.h"

#include <iterator>
#include <utility>

#include <geometry_msgs/Quaternion.h>
#include <ros/time.h>

namespace umigv {
namespace make_goal {
namespace detail {

static inline geodesy::UTMPoint wgs84_to_utm(
    const geographic_msgs::GeoPoint &point
) {
    geodesy::UTMPoint utm;
    geodesy::convert(point, utm);

    return utm;
}

static inline geometry_msgs::Quaternion identity_quaternion() noexcept {
    geometry_msgs::Quaternion quaternion;

    quaternion.x = 1.0;
    quaternion.y = 0.0;
    quaternion.z = 0.0;
    quaternion.w = 0.0;

    return quaternion;
}

static inline GoalT utm_to_goal(const geodesy::UTMPoint &point,
                                const std::string &id) {
    GoalT goal;

    goal.goal_id.id = id;
    goal.goal.target_pose.header.frame_id = "utm";
    goal.goal.target_pose.pose.position = geodesy::toGeometry(point);
    goal.goal.target_pose.pose.orientation = identity_quaternion();

    return goal;
}

} // namespace detail

GoalDirectorBuilder&
GoalDirectorBuilder::with_node(ros::NodeHandle &node) noexcept {
    node_ = &node;

    return *this;
}

GoalDirectorBuilder&
GoalDirectorBuilder::from_stream(std::istream &is) noexcept {
    const std::istream_iterator<geographic_msgs::GeoPoint> first{ is };
    const std::istream_iterator<geographic_msgs::GeoPoint> last;

    waypoints_.emplace();

    std::transform(first, last, std::back_inserter(waypoints_.value()),
                   &detail::wgs84_to_utm);

    return *this;
}

GoalDirectorBuilder&
GoalDirectorBuilder::with_threshold(const f64 threshold) noexcept {
    threshold_ = threshold;

    return *this;
}

GoalDirectorBuilder&
GoalDirectorBuilder::with_goal_id(std::string id) noexcept {
    id_ = std::move(id);

    return *this;
}

GoalDirector GoalDirectorBuilder::build() {
    if (!node_ || !waypoints_ || !threshold_ || !id_) {
        throw std::logic_error{ "GoalDirectorBuilder::build" };
    }

    return { node_->advertise<GoalT>("move_base/goal", 10),
             std::move(waypoints_.value()), threshold_.value(),
             std::move(id_.value()) };
}

GoalDirector::GoalDirector(GoalDirector &&other) noexcept
: publisher_{ std::move(other.publisher_) },
  waypoints_{ std::move(other.waypoints_) },
  distance_threshold_{ other.distance_threshold_ },
  current_iter_{ std::move(other.current_iter_) }, seq_{ other.seq_ },
  id_{ std::move(other.id_) } { }

void GoalDirector::update_fix(
    const sensor_msgs::NavSatFix::ConstPtr &fix_ptr
) {
    if (!current()) {
        return;
    }

    if (is_goal_reached(*fix_ptr)) {
        ++current_iter_;
    }
}

void GoalDirector::publish_goal(const ros::TimerEvent&) {
    if (!current()) {
        return;
    }

    GoalT to_broadcast = detail::utm_to_goal(current().value(), id_);

    to_broadcast.header.seq = seq_;
    to_broadcast.goal.target_pose.header.seq = seq_;

    const auto now = ros::Time::now();
    to_broadcast.header.stamp = now;
    to_broadcast.goal.target_pose.header.stamp = now;
    

    publisher_.publish(to_broadcast);
    ++seq_;


    ROS_INFO_STREAM("published goal " << current_index().value());
}

GoalDirector::GoalDirector(ros::Publisher publisher,
                           std::vector<WaypointT> waypoints,
                           const f64 threshold, std::string id) noexcept
: publisher_{ std::move(publisher) }, waypoints_{ std::move(waypoints) },
  distance_threshold_{ threshold }, id_{ std::move(id) } { }

bool GoalDirector::is_goal_reached(const sensor_msgs::NavSatFix &fix)
const noexcept {
    if (!current()) {
        return false;
    }

    geodesy::UTMPoint nav_utm;
    geodesy::convert(fix, nav_utm);

    const geometry_msgs::Point nav_point = geodesy::toGeometry(nav_utm);
    const geometry_msgs::Point waypoint_point =
        geodesy::toGeometry(current().value());

    const f64 distance_from_waypoint =
        planar_distance(nav_point, waypoint_point);

    ROS_DEBUG_STREAM("current UTM coordinates: " << nav_point);
    ROS_DEBUG_STREAM("waypoint UTM coordinates: " << waypoint_point);
    ROS_DEBUG_STREAM("distance from waypoint: " << distance_from_waypoint);

    return distance_from_waypoint <= distance_threshold_;

}

boost::optional<std::reference_wrapper<const WaypointT>>
GoalDirector::current() const noexcept {
    if (current_iter_ < waypoints_.cbegin()
        || current_iter_ >= waypoints_.cend()) {
        return boost::none;
    }

    using ReturnT = boost::optional<std::reference_wrapper<const WaypointT>>;
    return ReturnT{ std::cref(*current_iter_) };
}

boost::optional<std::size_t> GoalDirector::current_index() const noexcept {
    if (!current()) {
        return boost::none;
    }

    const auto index = std::distance(waypoints_.cbegin(), current_iter_);

    using ReturnT = boost::optional<std::size_t>;
    return ReturnT{ static_cast<std::size_t>(index) };
}

} // namespace make_goal
} // namespace umigv
