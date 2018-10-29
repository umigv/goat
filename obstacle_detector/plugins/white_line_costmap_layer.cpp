#include "obstacle_detector/white_line_costmap_layer.h"

#include <boost/optional.hpp>

#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <umigv_utilities/ros.hpp>

PLUGINLIB_EXPORT_CLASS(obstacle_detector::WhiteLineCostmapLayer,
                       costmap_2d::Layer);

namespace obstacle_detector {
namespace detail {

static inline tf2::Vector3 as_tf2(const geometry_msgs::Point &point) {
    return { point.x, point.y, point.z };
}

static inline tf2::Vector3 as_tf2(const geometry_msgs::Vector3 &vector) {
    return { vector.x, vector.y, vector.z };
}

static inline tf2::Quaternion as_tf2(const geometry_msgs::Quaternion &quat) {
    return { quat.x, quat.y, quat.z, quat.w };
}

static inline tf2::Transform as_tf2(const geometry_msgs::Transform &transform) {
    return tf2::Transform{ as_tf2(transform.rotation),
                           as_tf2(transform.translation) };
}

static inline geometry_msgs::Point as_msg(const tf2::Vector3 &vector) {
    geometry_msgs::Point point;

    point.x = vector.getX();
    point.y = vector.getY();
    point.z = vector.getZ();

    return point;
}

static geometry_msgs::Point transform_point(const geometry_msgs::Point &point,
                                            const tf2::Transform &transform) {
    const tf2::Vector3 point_tf2 = as_tf2(point);
    const tf2::Vector3 transformed = transform * point_tf2;

    return as_msg(transformed);
}

} // namespace detail

void WhiteLineCostmapLayer::add_points(const goat_msgs::PointArrayStamped::ConstPtr &points_ptr) {
    TransformReturn ret;

    try {
        ret = transform_points(*points_ptr);
    } catch (const tf2::TransformException &e) {
        return;
    }

    for (const auto &index : ret.indices) {
        costmap_2d::Costmap2D::setCost(index.i, index.j,
                                       costmap_2d::LETHAL_OBSTACLE);
    }

    costmap_2d::CostmapLayer::addExtraBounds(
        ret.lower_bound.x, ret.lower_bound.y,
        ret.upper_bound.x, ret.upper_bound.y
    );
}

void WhiteLineCostmapLayer::activate() { }

void WhiteLineCostmapLayer::deactivate() { }

void WhiteLineCostmapLayer::reset() { }

void WhiteLineCostmapLayer::updateBounds(
    const double robot_x, const double robot_y, double, // yaw not useful
    double *const min_x, double *const min_y,
    double *const max_x, double *const max_y
) {
    costmap_2d::Costmap2D::updateOrigin(
        robot_x - costmap_2d::Costmap2D::getSizeInMetersX() / 2.0,
        robot_y - costmap_2d::Costmap2D::getSizeInMetersY() / 2.0
    );
    costmap_2d::CostmapLayer::useExtraBounds(min_x, min_y, max_x, max_y);

    const umigv::Point2 world_min = map_to_world({ 0, 0 });
    const umigv::Point2 world_max = map_to_world(size());

    *min_x = std::min(*min_x, world_min.x);
    *min_y = std::min(*min_y, world_min.y);

    *max_x = std::max(*max_x, world_max.x);
    *max_y = std::max(*max_y, world_max.y);
}

void WhiteLineCostmapLayer::updateCosts(costmap_2d::Costmap2D &master_grid,
                                        const int min_i, const int min_j,
                                        const int max_i, const int max_j) {
    costmap_2d::CostmapLayer::updateWithMax(master_grid, min_i, min_j,
                                            max_i, max_j);
}

void WhiteLineCostmapLayer::matchSize() {
    costmap_2d::CostmapLayer::matchSize();
}

IndexPair WhiteLineCostmapLayer::size() const {
    return { costmap_2d::Costmap2D::size_x_, costmap_2d::Costmap2D::size_y_ };
}

void WhiteLineCostmapLayer::onInitialize() {
    static constexpr double DEFAULT_TRANSFORM_TIMEOUT = 0.1;

    costmap_2d::CostmapLayer::onInitialize();

    umigv::ParameterServer params{ false, "~/" + name_ };
    ros::NodeHandle node;

    costmap_2d::Layer::current_ = true;
    costmap_2d::Layer::enabled_ = true;
    matchSize();

    frame_ = costmap_2d::Layer::layered_costmap_->getGlobalFrameID();

    const double transform_timeout =
        params["transform_timeout"].value_or(DEFAULT_TRANSFORM_TIMEOUT);

    if (!std::isfinite(transform_timeout) || transform_timeout <= 0.0) {
        ROS_WARN_STREAM("WhiteLineCostmapLayer::onInitialize: invalid "
                        "~transform_timeout, using default value of "
                        << DEFAULT_TRANSFORM_TIMEOUT);

        transform_timeout_ = ros::Duration{ DEFAULT_TRANSFORM_TIMEOUT };
    } else {
        ROS_INFO_STREAM("WhiteLineCostmapLayer::onInitialize: "
                        "~transform_timeout = " << transform_timeout);
        transform_timeout_ = ros::Duration{ transform_timeout };
    }

    points_subscriber_ = node.subscribe<goat_msgs::PointArrayStamped>(
        "camera/white_line_points", 10,
        &WhiteLineCostmapLayer::add_points, this
    );

    ROS_INFO_STREAM("WhiteLineCostmapLayer::onInitialize: subscribed to "
                    << points_subscriber_.getTopic());
}

WhiteLineCostmapLayer::TransformReturn WhiteLineCostmapLayer::transform_points(
    const goat_msgs::PointArrayStamped &points
) const {
    const auto &target = frame_;
    const auto &source = points.header.frame_id;

    TransformReturn to_return;
    tf2::Transform transform;

    try {
        const auto transform_msg =
            buffer_.lookupTransform(target, source, { 0, 0 },
                                    transform_timeout_);

        transform = detail::as_tf2(transform_msg.transform);
    } catch (const tf2::TransformException &e) {
        ROS_WARN_STREAM("WhiteLineCostmapLayer::transform_points: unable to "
                        "lookup transform from " << target << " to " << source);

        return to_return;
    }

    to_return.indices.reserve(points.points.size());

    boost::optional<umigv::Point2> world_min;
    boost::optional<umigv::Point2> world_max;
    for (const auto &point : points.points) {
        const auto transformed = detail::transform_point(point, transform);
        const auto world_coords = umigv::Point2::from_geometry(transformed);
        const auto map_indices = world_to_map(world_coords);

        if (!map_indices.is_in_bounds({ 0, 0 }, size())) {
            continue;
        }

        if (!world_min || !world_max) {
            world_min = world_coords;
            world_max = world_coords;
        } else {
            world_min = min(world_min.value(), world_coords);
            world_max = max(world_max.value(), world_coords);
        }

        to_return.indices.push_back(map_indices);
    }

    if (world_min && world_max) {
        to_return.lower_bound = world_min.value();
        to_return.upper_bound = world_max.value();
    }

    return to_return;
}

umigv::Point2 WhiteLineCostmapLayer::map_to_world(
    const IndexPair &indices
) const {
    umigv::Point2 output;
    costmap_2d::Costmap2D::mapToWorld(indices.i, indices.j, output.x, output.y);

    return output;
}

IndexPair WhiteLineCostmapLayer::world_to_map(
    const umigv::Point2 &point
) const {
    IndexPair output;
    costmap_2d::Costmap2D::worldToMap(point.x, point.y, output.i, output.j);

    return output;
}

}  // namespace obstacle_detector
