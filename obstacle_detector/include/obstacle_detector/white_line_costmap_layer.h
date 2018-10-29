#ifndef UMIGV_OBSTACLE_DETECTOR_WHITE_LINE_COSTMAP_LAYER_H
#define UMIGV_OBSTACLE_DETECTOR_WHITE_LINE_COSTMAP_LAYER_H

#include "point2.hpp"

#include <cassert>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include <costmap_2d/costmap_layer.h>
#include <goat_msgs/PointArrayStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace obstacle_detector {

struct IndexPair {
    unsigned i = 0;
    unsigned j = 0;

    constexpr IndexPair() noexcept = default;

    constexpr IndexPair(const unsigned i_val, const unsigned j_val) noexcept
    : i{ i_val }, j{ j_val } { }

    constexpr static IndexPair from_signed(const int i_val, const int j_val) {
        assert(i_val >= 0 && j_val >= 0);

        return { static_cast<unsigned>(i_val), static_cast<unsigned>(j_val) };
    }

    constexpr bool is_in_bounds(const IndexPair min,
                                const IndexPair max) const {
        assert(min.i <= max.i && min.j <= max.j);
        return i >= min.i && i < max.i && j >= min.j && j < max.j;
    }
};

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const IndexPair &indices) {
    return os << '[' << indices.i << ' ' << indices.j << ']';
}

class WhiteLineCostmapLayer : public costmap_2d::CostmapLayer {
public:
    virtual ~WhiteLineCostmapLayer() = default;

    void add_points(const goat_msgs::PointArrayStamped::ConstPtr &points_ptr);

    // costmap_2d::Layer functions

    void activate() override;

    void deactivate() override;

    void reset() override;

    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                      double *min_x, double *min_y,
                      double *max_x, double *max_y) override;

    void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
                     int max_i, int max_j) override;

    void matchSize() override;

protected:
    // costmap_2d::Layer functions

    void onInitialize() override;

private:
    struct TransformReturn {
        std::vector<IndexPair> indices;
        umigv::Point2 lower_bound;
        umigv::Point2 upper_bound;
    };

    TransformReturn transform_points(
        const goat_msgs::PointArrayStamped &points
    ) const;

    umigv::Point2 map_to_world(const IndexPair &point) const;

    IndexPair world_to_map(const umigv::Point2 &point) const;

    IndexPair size() const;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener{ buffer_ };
    ros::Duration transform_timeout_{ 0.1 };
    ros::Subscriber points_subscriber_;
    std::string frame_;
};

} // namespace obstacle_detector

#endif
