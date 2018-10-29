#ifndef UMIGV_MAKE_GOAL_MESSAGE_UTILS_H
#define UMIGV_MAKE_GOAL_MESSAGE_UTILS_H

#include <iostream>
#include <utility>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <ros/time.h>
#include <sensor_msgs/NavSatFix.h>
#include <umigv_utilities/types.hpp>

namespace geometry_msgs {

std::ostream& operator<<(std::ostream &os, const Point &point);

std::ostream& operator<<(std::ostream &os, const Quaternion &quat);

std::ostream& operator<<(std::ostream &os, const Pose &pose);

std::istream& operator>>(std::istream &is, Point &point);

std::istream& operator>>(std::istream &is, Quaternion &quat);

std::istream& operator>>(std::istream &is, Pose &pose);

Point operator-(const Point &point) noexcept;

Point operator+(const Point &lhs, const Point &rhs) noexcept;

Point operator-(const Point &lhs, const Point &rhs) noexcept;

} // namespace geometry_msgs

namespace move_base_msgs {

std::ostream& operator<<(std::ostream &os, const MoveBaseActionGoal &goal);

std::istream& operator>>(std::istream &is, MoveBaseActionGoal &goal);

} // namespace move_base_msgs

namespace sensor_msgs {
    std::ostream& operator<<(std::ostream &os, const sensor_msgs::NavSatFix &nav);
    std::istream& operator>>(std::istream &is, sensor_msgs::NavSatFix &nav);
}

namespace geographic_msgs {

std::ostream& operator<<(std::ostream &os, const GeoPoint &point);

std::istream& operator>>(std::istream &is, GeoPoint &point);

} // namespace geographic_msgs

namespace geodesy {

static constexpr inline void
fromMsg(const sensor_msgs::NavSatFix &fix,
        geographic_msgs::GeoPoint &point) noexcept {
    point.latitude = fix.latitude;
    point.longitude = fix.longitude;
    point.altitude = fix.altitude;
}

static inline void fromMsg(const sensor_msgs::NavSatFix &fix, UTMPoint &point) {
    geographic_msgs::GeoPoint converted;

    convert(fix, converted);
    convert(converted, point);
}

} // namespace geodesy

namespace umigv {
namespace make_goal {

f64 dot(const geometry_msgs::Point &lhs,
        const geometry_msgs::Point &rhs) noexcept;

f64 norm(const geometry_msgs::Point &point) noexcept;

f64 distance(const geometry_msgs::Point &lhs,
             const geometry_msgs::Point &rhs) noexcept;

f64 distance(const sensor_msgs::NavSatFix &lhs, 
             const sensor_msgs::NavSatFix &rhs) noexcept;

f64 planar_distance(const geometry_msgs::Point &lhs,
                    const geometry_msgs::Point &rhs) noexcept;

} // namespace make_goal
} // namespace umigv

#endif
