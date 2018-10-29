#ifndef UMIGV_IMU_TRANSFORMER_H
#define UMIGV_IMU_TRANSFORMER_H

#include <ros/ros.h> // ros::Publisher
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <string>
#include <tuple>
#include <type_traits> // metaprogramming

namespace umigv {

class ImuTransformer {
    using DataTupleT = std::tuple<ros::Publisher, ros::Publisher, std::string>;

public:
    ImuTransformer() = delete;

    ImuTransformer(const ImuTransformer &other) = default;

    ImuTransformer(ImuTransformer &&other) noexcept = default;

    template <typename ...Args,
              typename = std::enable_if_t<
                  std::is_constructible<DataTupleT, Args...>::value
              >>
    ImuTransformer(Args &&...args)
    noexcept(std::is_nothrow_constructible<DataTupleT, Args...>::value)
    : data_{ std::forward<Args>(args)... } { }

    ImuTransformer& operator=(const ImuTransformer &other) = default;

    ImuTransformer& operator=(ImuTransformer &&other) = default;

    void transform_imu(const sensor_msgs::Imu::ConstPtr &imu_ptr);

    void transform_magnetic_field(
        const sensor_msgs::MagneticField::ConstPtr &field_ptr
    );

private:
    inline ros::Publisher& imu_publisher() noexcept {
        return std::get<0>(data_);
    }

    inline const ros::Publisher& imu_publisher() const noexcept {
        return std::get<0>(data_);
    }

    inline ros::Publisher& field_publisher() noexcept {
        return std::get<1>(data_);
    }

    inline const ros::Publisher& field_publisher() const noexcept {
        return std::get<1>(data_);
    }

    inline std::string& frame_id() noexcept {
        return std::get<2>(data_);
    }

    inline const std::string& frame_id() const noexcept {
        return std::get<2>(data_);
    }

    std_msgs::Header transform_header(const std_msgs::Header &header) const;

    DataTupleT data_;
};

} // namespace umigv

#endif
