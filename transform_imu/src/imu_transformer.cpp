#include "imu_transformer.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // tf2::fromMsg, tf2::toMsg
#include <umigv_utilities/types.hpp>

using namespace umigv::types;

// [x, y, z] -> [y, x, -z]
static const tf2::Transform TRANSFORM{ tf2::Matrix3x3{ 0,  1,  0,
                                                       1,  0,  0,
                                                       0,  0, -1 }  };

static inline tf2::Quaternion
from_message(const geometry_msgs::Quaternion &quaternion) {
   tf2::Quaternion converted;

   tf2::fromMsg(quaternion, converted);

   return converted;
}

static inline tf2::Vector3
from_message(const geometry_msgs::Vector3 &vector) {
    tf2::Vector3 converted;

    tf2::fromMsg(vector, converted);

    return converted;
}

static inline geometry_msgs::Vector3
transform_vector3(const geometry_msgs::Vector3 &vector) {
    tf2::Vector3 converted = from_message(vector);

    converted = TRANSFORM * converted;

    return tf2::toMsg(converted);
}

static inline geometry_msgs::Quaternion
transform_quaternion(const geometry_msgs::Quaternion &quaternion) {
    tf2::Quaternion converted = from_message(quaternion);

    converted = TRANSFORM * converted;

    return tf2::toMsg(converted);
}

// covariance is always postive
// 0 1 2    4 3 5
// 3 4 5 -> 1 0 2
// 6 7 8    7 6 8
static inline boost::array<f64, 9>
transform_matrix(const boost::array<f64, 9> &matrix) {
    return boost::array<f64, 9>{ { matrix[4], matrix[3], matrix[5],
                                   matrix[1], matrix[0], matrix[2],
                                   matrix[7], matrix[6], matrix[8] } };
}

namespace umigv {

void ImuTransformer::transform_imu(const sensor_msgs::Imu::ConstPtr &imu_ptr) {
    const sensor_msgs::Imu &imu = *imu_ptr;

    sensor_msgs::Imu transformed;

    transformed.header = transform_header(imu.header);

    transformed.orientation = transform_quaternion(imu.orientation);
    transformed.orientation_covariance =
        transform_matrix(imu.orientation_covariance);

    transformed.angular_velocity = transform_vector3(imu.angular_velocity);
    transformed.angular_velocity_covariance =
        transform_matrix(imu.angular_velocity_covariance);

    transformed.linear_acceleration =
        transform_vector3(imu.linear_acceleration);
    transformed.linear_acceleration_covariance =
        transform_matrix(imu.linear_acceleration_covariance);

    imu_publisher().publish(transformed);
}

void ImuTransformer::transform_magnetic_field(
    const sensor_msgs::MagneticField::ConstPtr &field_ptr
) {
    const sensor_msgs::MagneticField &field = *field_ptr;

    sensor_msgs::MagneticField transformed;

    transformed.header = transform_header(field.header);
    transformed.magnetic_field = transform_vector3(field.magnetic_field);
    transformed.magnetic_field_covariance =
        transform_matrix(field.magnetic_field_covariance);

    field_publisher().publish(transformed);
}

std_msgs::Header
ImuTransformer::transform_header(const std_msgs::Header &header) const {
    std_msgs::Header transformed;

    transformed.seq = header.seq;
    transformed.stamp = header.stamp;
    transformed.frame_id = frame_id();

    return transformed;
}

} // namespace umigv
