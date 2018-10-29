#include "message_utils.h"

#include <cmath>

namespace geometry_msgs {

std::ostream& operator<<(std::ostream &os, const Point &point) {
    return os << point.x << ' ' << point.y << ' ' << point.z;
}

std::ostream& operator<<(std::ostream &os, const Quaternion &quat) {
    return os << quat.x << ' ' << quat.y << ' ' << quat.z << ' ' << quat.w;
}

std::ostream& operator<<(std::ostream &os, const Pose &pose) {
    return os << pose.position << ' ' << pose.orientation;
}

std::istream& operator>>(std::istream &is, Point &point) {
	return is >> point.x >> point.y >> point.z;
}

std::istream& operator>>(std::istream &is, Quaternion &quat) {
	return is >> quat.x >> quat.y >> quat.z >> quat.w;
}

std::istream& operator>>(std::istream &is, Pose &pose) {
	return is >> pose.position >> pose.orientation;
}

Point operator-(const Point &point) noexcept {
    Point negated;

    negated.x = -point.x;
    negated.y = -point.y;
    negated.z = -point.z;

    return negated;
}

Point operator+(const Point &lhs, const Point &rhs) noexcept {
    Point sum;

    sum.x = lhs.x + rhs.x;
    sum.y = lhs.y + rhs.y;
    sum.z = lhs.z + rhs.z;

    return sum;
}

Point operator-(const Point &lhs, const Point &rhs) noexcept {
    Point difference;

    difference.x = lhs.x - rhs.x;
    difference.y = lhs.y - rhs.y;
    difference.z = lhs.z - rhs.z;

	return difference;
}

} // namespace geometry_msgs

namespace move_base_msgs {

std::ostream& operator<<(std::ostream &os, const MoveBaseActionGoal &goal) {
    return os << goal.goal.target_pose.pose;
}

std::istream& operator>>(std::istream &is, MoveBaseActionGoal &goal) {
    return is >> goal.goal.target_pose.pose;
}

} // namespace move_base_msgs

namespace sensor_msgs {

std::ostream& operator<<(std::ostream &os, const NavSatFix &nav) {
    return os << nav.latitude << ' ' << nav.longitude << ' ' << nav.altitude;
}

std::istream& operator>>(std::istream &is, NavSatFix &nav) {
    return is >> nav.latitude >> nav.longitude >> nav.altitude;
}

} // namespace sensor_msgs

namespace geographic_msgs {

std::ostream& operator<<(std::ostream &os, const GeoPoint &point) {
    return os << point.latitude << ' ' << point.longitude << ' '
              << point.altitude;
}

std::istream& operator>>(std::istream &is, GeoPoint &point) {
    return is >> point.latitude >> point.longitude >> point.altitude;
}

} // namespace geographic_msgs

namespace umigv {
namespace make_goal {

f64 dot(const geometry_msgs::Point &lhs,
        const geometry_msgs::Point &rhs) noexcept {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

f64 norm(const geometry_msgs::Point &point) noexcept {
    return std::sqrt(dot(point, point));
}

f64 distance(const geometry_msgs::Point &lhs,
             const geometry_msgs::Point &rhs) noexcept {
    return norm(lhs - rhs);
}

f64 distance(const sensor_msgs::NavSatFix &lhs, 
             const sensor_msgs::NavSatFix &rhs) noexcept {

    double absdist_long = abs(lhs.longitude - rhs.longitude);

    double num = sqrt(cos(rhs.latitude) * pow(sin(absdist_long), 2) 
        + (cos(lhs.latitude) * sin(rhs.latitude) 
        - sin(lhs.latitude) * cos(rhs.latitude) * pow(cos(absdist_long), 2)));

    double denom = sin(lhs.latitude) * sin(rhs.latitude) + cos(lhs.latitude) * cos(rhs.latitude) * cos(absdist_long);

    return atan2(num, denom);
}

f64 planar_distance(const geometry_msgs::Point &lhs,
                    const geometry_msgs::Point &rhs) noexcept {
    geometry_msgs::Point lhs_planar;

    lhs_planar.x = lhs.x;
    lhs_planar.y = lhs.y;
    lhs_planar.z = 0.0;

    geometry_msgs::Point rhs_planar;

    rhs_planar.x = rhs.x;
    rhs_planar.y = rhs.y;
    rhs_planar.z = 0.0;

    return distance(lhs_planar, rhs_planar);
}

} // namespace make_goal
} // namespace umigv
