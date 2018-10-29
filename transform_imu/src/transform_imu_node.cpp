#include "imu_transformer.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <umigv_utilities/rosparam.hpp>

#include <boost/array.hpp>

#include <utility> // std::move

int main(int argc, char *argv[]) {
    using namespace std::literals;

    ros::init(argc, argv, "transform_imu_node");
    ros::NodeHandle handle;
    ros::NodeHandle private_handle{ "~" };

    auto frame_id = umigv::get_parameter_or(private_handle, "frame_id",
                                            "imu"s);
    auto imu_publisher =
        handle.advertise<sensor_msgs::Imu>("imu/data_raw_enu", 10);
    auto magnetic_field_publisher =
        handle.advertise<sensor_msgs::MagneticField>("imu/mag_enu", 10);

    umigv::ImuTransformer transformer{
        std::move(imu_publisher), std::move(magnetic_field_publisher),
        std::move(frame_id)
    };

    const auto imu_handler = &umigv::ImuTransformer::transform_imu;
    const auto magnetic_field_handler =
        &umigv::ImuTransformer::transform_magnetic_field;

    auto imu_subscriber = handle.subscribe<sensor_msgs::Imu>(
        "imu/data_raw", 10, imu_handler, &transformer
    );
    auto magnetic_field_subscriber =
        handle.subscribe<sensor_msgs::MagneticField>(
            "imu/mag", 10, magnetic_field_handler, &transformer
        );

    ros::spin();
}
