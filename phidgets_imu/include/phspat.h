#ifndef PHSPAT_H
#define PHSPAT_H

#include <cstdint>
#include <array>
#include <chrono>
#include <memory>
#include <shared_mutex>
#include <system_error>

#include <boost/optional.hpp>

#include <phidget22.h>

namespace ph {

class Spatial;

namespace spatial {

struct Config {
    int channel{ PHIDGET_CHANNEL_ANY };
    std::int32_t serial_number{ PHIDGET_SERIALNUMBER_ANY };
    boost::optional<std::string> label;
};

class HandleBase;

template <typename OnData, typename OnAttach, typename OnError>
class Handle;

class Clock {
public:
    friend Spatial;

    using rep = std::uint32_t;
    using period = std::milli;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<Clock>;

    time_point now() const;

private:
    explicit Clock(std::shared_ptr<HandleBase> handle_ptr) noexcept;

    std::shared_ptr<HandleBase> handle_ptr_;
};

struct Vector {
    double x;
    double y;
    double z;
};

struct Data {
    Vector linear_acceleration; // m/s^2
    Vector angular_velocity; // rad/s
    Vector magnetic_field; // tesla (T)
    Clock::time_point timestamp;
};

class DataIntervalReference;

} // namespace spatial

class Spatial {
public:
    friend spatial::DataIntervalReference;

    template <typename OnData, typename OnAttach, typename OnError>
    friend class spatial::Handle;

    Spatial();

    template <typename OnData, typename OnAttach, typename OnError>
    Spatial(const OnData &on_data, const OnAttach &on_attach, const OnError &on_error);

    template <typename OnData, typename OnAttach, typename OnError>
    Spatial(const spatial::Config &config, const OnData &on_data,
            const OnAttach &on_attach, const OnError &on_error);

    Spatial(const Spatial &other) = default;

    Spatial& operator=(const Spatial &other) = default;

    spatial::DataIntervalReference data_interval() noexcept;

    spatial::Clock::duration data_interval() const;

    spatial::Clock::duration min_data_interval() const;

    spatial::Clock::duration max_data_interval() const;

    spatial::Clock clock() const;

private:
    explicit Spatial(std::shared_ptr<spatial::HandleBase> handle) noexcept;

    Spatial(Spatial &&other) = default;

    Spatial& operator=(Spatial &&other) = default;

    std::shared_ptr<spatial::HandleBase> handle_ptr_;
};

const std::error_category& phidget_category() noexcept;

const std::error_category& phidget_error_event_category(const char *description);

namespace spatial {

class DataIntervalReference {
public:
    friend Spatial;

    DataIntervalReference(const DataIntervalReference &other) = default;

    DataIntervalReference& operator=(Clock::duration data_interval);

    DataIntervalReference& operator=(const DataIntervalReference &data_interval);

    operator Clock::duration() const;

private:
    explicit DataIntervalReference(Spatial spatial) noexcept;

    Spatial spatial_;
};

class HandleBase : public std::enable_shared_from_this<HandleBase> {
public:
    HandleBase(const spatial::Config &config, PhidgetSpatial_OnSpatialDataCallback on_data,
               Phidget_OnAttachCallback on_attach, Phidget_OnErrorCallback on_error);

    virtual ~HandleBase();

    void open();

    void close();

    void open_accelerometer();

    void close_accelerometer();

    Clock::duration get_min_data_interval() const;

    Clock::duration get_max_data_interval() const;

    Clock::duration get_data_interval() const;

    void set_data_interval(Clock::duration data_interval);

    Clock::time_point get_timestamp() const;

protected:
    mutable std::mutex mtx_;

private:
    PhidgetHandle as_phidget() const noexcept;

    PhidgetHandle accel_as_phidget() const noexcept;

    PhidgetSpatialHandle spatial_;
    PhidgetAccelerometerHandle accelerometer_;
};

Data make_data(const double lin[3], const double ang[3],
               const double mag[3], double time) noexcept;

template <typename OnData, typename OnAttach, typename OnError>
class Handle : public HandleBase {
public:
    Handle(const Config &config, const OnData &on_data,
           const OnAttach &on_attach, const OnError &on_error)
    : HandleBase{ config, &Handle::on_data, &Handle::on_attach, &Handle::on_error },
      on_data_{ on_data }, on_attach_{ on_attach }, on_error_{ on_error } { }

    virtual ~Handle() = default;

private:
    static void on_data(PhidgetSpatialHandle, void *handle_ptr_v,
                        const double linear_acceleration[3], const double angular_velocity[3],
                        const double magnetic_field[3], double timestamp) {
        const auto data{ make_data(linear_acceleration, angular_velocity,
                                   magnetic_field, timestamp) };
        auto &handle{ *static_cast<Handle*>(handle_ptr_v) };
        Spatial spatial{ handle.shared_from_this() };

        handle.on_data_(spatial, data);
    }

    static void on_attach(PhidgetHandle, void *handle_ptr_v) {
        auto &handle{ *static_cast<Handle*>(handle_ptr_v) };
        Spatial spatial{ handle.shared_from_this() };

        handle.open_accelerometer();
        handle.on_attach_(spatial);
    }

    static void on_error(PhidgetHandle, void *handle_ptr_v, Phidget_ErrorEventCode code,
                         const char *description) {
        auto &handle{ *static_cast<Handle*>(handle_ptr_v) };
        Spatial spatial{ handle.shared_from_this() };
        std::error_condition condition{ code, phidget_error_event_category(description) };

        handle.on_error_(spatial, condition);
    }

    const OnData on_data_{ };
    const OnAttach on_attach_{ };
    const OnError on_error_{ };
};

} // namespace spatial

template <typename OnData, typename OnAttach, typename OnError>
Spatial::Spatial(const OnData &on_data, const OnAttach &on_attach, const OnError &on_error)
: Spatial{ spatial::Config{ }, on_data, on_attach, on_error } { }

template <typename OnData, typename OnAttach, typename OnError>
Spatial::Spatial(const spatial::Config &config, const OnData &on_data,
                 const OnAttach &on_attach, const OnError &on_error)
: handle_ptr_{ std::make_shared<spatial::Handle<OnData, OnAttach, OnError>>(
    config, on_data, on_attach, on_error
) } {
    handle_ptr_->open();
}

} // namespace ph

#endif
