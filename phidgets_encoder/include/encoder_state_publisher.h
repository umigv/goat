#ifndef UMIGV_ENCODER_STATE_PUBLISHER_H
#define UMIGV_ENCODER_STATE_PUBLISHER_H

#include <phidgets_api/encoder.h> // phidgets::Encoder
#include <ros/ros.h> // ros::Publisher, ros::TimerEvent
#include <sensor_msgs/JointState.h> // sensor_msgs::JointState
#include <umigv_utilities/exceptions.hpp> // umigv::PhidgetException
#include <umigv_utilities/types.hpp>

#include <string> // std::string, std::to_string

namespace umigv {

struct RobotCharacteristics {
    RobotCharacteristics() = default;

    RobotCharacteristics& with_frame(const std::string &frame);

    RobotCharacteristics& with_frame(std::string &&frame) noexcept;

    RobotCharacteristics& with_rads_per_tick(f64 radians) noexcept;

    RobotCharacteristics& with_serial_number(int serial) noexcept;

    std::string frame_id;
    f64 rads_per_tick;
    int serial_number;
};

class EncoderStatePublisher final : private phidgets::Encoder {
public:
    EncoderStatePublisher(ros::Publisher publisher,
                          RobotCharacteristics characteristics) noexcept;

    ~EncoderStatePublisher();

    void publish_state(const ros::TimerEvent &event = ros::TimerEvent{ });

private:
    struct EncoderState {
        int left_ticks = 0;
        int right_ticks = 0;
        ros::Time update_time = ros::Time(0.0);
    };

    void try_attach(int serial_number);

    EncoderState poll_encoders();

    sensor_msgs::JointState make_message(EncoderState next_state) const;

    void attachHandler() override;

    void detachHandler() override;

    void errorHandler(int error_code) override;

    void indexHandler(int index, int index_position) override;

    void positionChangeHandler(int index, int, int delta_position) override;

    std::string frame_id_;
    EncoderState state_;
    ros::Publisher publisher_;
    f64 rads_per_tick_;
    int serial_number_ = -1;
    std::string name_;
    bool is_connected_ = false;
};

} // namespace umigv

#endif
