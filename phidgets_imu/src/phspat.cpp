#include <phspat.h>

#include "err.h"

#include <cassert>
#include <string>
#include <utility>
#include <set>

namespace ph {

const std::error_category& phidget_category() noexcept {
    return PhidgetCategory::instance();
}

struct EventCompare {
    using is_transparent = std::true_type;

    constexpr bool operator()(const PhidgetErrorEventCategory &lhs,
                              const PhidgetErrorEventCategory &rhs) const noexcept {
        return lhs.compare(rhs) < 0;
    }

    constexpr bool operator()(const PhidgetErrorEventCategory &lhs,
                              const char *rhs) const noexcept {
        return lhs.compare(rhs) < 0;
    }

    constexpr bool operator()(const char *lhs,
                              const PhidgetErrorEventCategory &rhs) const noexcept {
        return rhs.compare(lhs) > 0;
    }
};

const std::error_category& phidget_error_event_category(const char *description) {
    static std::mutex categories_mtx;
    static std::set<PhidgetErrorEventCategory, EventCompare> categories;

    const std::lock_guard<std::mutex> lck{ categories_mtx };

    const auto category_iter = categories.lower_bound(description);

    if (category_iter == categories.end() || category_iter->compare(description) != 0) {
        return *categories.emplace_hint(category_iter, description);
    }

    return *category_iter;
}

spatial::DataIntervalReference Spatial::data_interval() noexcept {
    return spatial::DataIntervalReference{ *this };
}

spatial::Clock::duration Spatial::data_interval() const {
    return handle_ptr_->get_data_interval();
}

spatial::Clock::duration Spatial::min_data_interval() const {
    return handle_ptr_->get_min_data_interval();
}

spatial::Clock::duration Spatial::max_data_interval() const {
    return handle_ptr_->get_max_data_interval();
}

spatial::Clock Spatial::clock() const {
    return spatial::Clock{ handle_ptr_ };
}

Spatial::Spatial(std::shared_ptr<spatial::HandleBase> handle_ptr) noexcept
: handle_ptr_{ std::move(handle_ptr) } {
    assert(handle_ptr_);
}

namespace spatial {

static void expect(PhidgetReturnCode ret, const char *what) {
    assert(what);

    if (ret == EPHIDGET_OK) {
        return;
    }

    throw std::system_error{ static_cast<int>(ret), phidget_category(), what };
}

Clock::time_point Clock::now() const {
    return handle_ptr_->get_timestamp();
}

Clock::Clock(std::shared_ptr<HandleBase> handle_ptr) noexcept
: handle_ptr_{ std::move(handle_ptr) } { }

static void on_detach(PhidgetHandle, void *handle_v) {
    HandleBase &handle{ *static_cast<HandleBase*>(handle_v) };
    handle.close_accelerometer();
}

HandleBase::HandleBase(const spatial::Config &config,
                       PhidgetSpatial_OnSpatialDataCallback on_data,
                       Phidget_OnAttachCallback on_attach, Phidget_OnErrorCallback on_error) {
    const auto expect = [](PhidgetReturnCode ret) {
        ph::spatial::expect(ret, "ph::spatial::HandleBase::HandleBase()");
    };

    expect(PhidgetSpatial_create(&spatial_));

    try {
        expect(Phidget_setChannel(as_phidget(), config.channel));
        expect(Phidget_setDeviceSerialNumber(as_phidget(), config.serial_number));

        if (config.label) {
            expect(Phidget_setDeviceLabel(as_phidget(), config.label->c_str()));
        } else {
            expect(Phidget_setDeviceLabel(as_phidget(), PHIDGET_LABEL_ANY));
        }

        expect(PhidgetSpatial_setOnSpatialDataHandler(spatial_, on_data, this));
        expect(Phidget_setOnAttachHandler(as_phidget(), on_attach, this));
        expect(Phidget_setOnDetachHandler(as_phidget(), on_detach, this));
        expect(Phidget_setOnErrorHandler(as_phidget(), on_error, this));

        expect(PhidgetAccelerometer_create(&accelerometer_));
    } catch (...) {
        expect(PhidgetSpatial_delete(&spatial_));

        throw;
    }
}

HandleBase::~HandleBase() {
    const auto expect = [](PhidgetReturnCode ret) {
        ph::spatial::expect(ret, "ph::spatial::HandleBase::~HandleBase()");
    };

    Phidget_close(accel_as_phidget()); // may not be open
    expect(PhidgetAccelerometer_delete(&accelerometer_));

    Phidget_close(as_phidget()); // may not be open
    expect(PhidgetSpatial_delete(&spatial_));
}

void HandleBase::open() {
    expect(Phidget_openWaitForAttachment(as_phidget(), PHIDGET_TIMEOUT_INFINITE),
           "ph::spatial::HandleBase::open");
    // attachment will call open_accelerometer
}

void HandleBase::open_accelerometer() {
    const auto expect = [](PhidgetReturnCode ret) {
        ph::spatial::expect(ret, "ph::spatial::HandleBase::open_accelerometer");
    };

    const std::unique_lock<std::mutex> lck{ mtx_ };

    int channel;
    expect(Phidget_getChannel(as_phidget(), &channel));

    expect(Phidget_setChannel(accel_as_phidget(), channel));
    expect(Phidget_openWaitForAttachment(accel_as_phidget(), PHIDGET_TIMEOUT_INFINITE));
}

void HandleBase::close_accelerometer() {
    const std::unique_lock<std::mutex> lck{ mtx_ };

    expect(Phidget_close(accel_as_phidget()), "ph::spatial::HandleBase::close_accelerometer");
}

Clock::duration HandleBase::get_min_data_interval() const {
    const std::lock_guard<std::mutex> lck{ mtx_ };

    std::uint32_t min_data_interval;
    expect(PhidgetSpatial_getMinDataInterval(spatial_, &min_data_interval),
           "ph::spatial::HandleBase::get_min_data_interval");

    return Clock::duration{ min_data_interval };
}

Clock::duration HandleBase::get_max_data_interval() const {
    const std::lock_guard<std::mutex> lck{ mtx_ };

    std::uint32_t max_data_interval;
    expect(PhidgetSpatial_getMaxDataInterval(spatial_, &max_data_interval),
           "ph::spatial::HandleBase::get_max_data_interval");

    return Clock::duration{ max_data_interval };
}

Clock::duration HandleBase::get_data_interval() const {
    const std::lock_guard<std::mutex> lck{ mtx_ };

    std::uint32_t data_interval;
    expect(PhidgetSpatial_getDataInterval(spatial_, &data_interval),
           "ph::spatial::HandleBase::get_data_interval");

    return Clock::duration{ data_interval };;
}

void HandleBase::set_data_interval(Clock::duration data_interval) {
    const std::lock_guard<std::mutex> lck{ mtx_ };

    expect(PhidgetSpatial_setDataInterval(spatial_, data_interval.count()),
           "ph::spatial::HandleBase::set_data_interval");
}

Clock::time_point HandleBase::get_timestamp() const {
    const std::lock_guard<std::mutex> lck{ mtx_ };

    double timestamp_ms;
    expect(PhidgetAccelerometer_getTimestamp(accelerometer_, &timestamp_ms),
           "ph::spatial::HandleBase::get_timestamp");
    const std::chrono::duration<double, std::milli> timestamp{ timestamp_ms };

    return Clock::time_point{ std::chrono::duration_cast<Clock::duration>(timestamp) };
}

PhidgetHandle HandleBase::as_phidget() const noexcept {
    return reinterpret_cast<PhidgetHandle>(spatial_);
}

PhidgetHandle HandleBase::accel_as_phidget() const noexcept {
    return reinterpret_cast<PhidgetHandle>(accelerometer_);
}

Data make_data(const double lin[3], const double ang[3],
               const double mag[3], double stamp) noexcept {
    constexpr double MS2_PER_G{ 9.80665 };
    constexpr double RADS_PER_DEG{ 0.017453292519943295769 };
    constexpr double TESLA_PER_GAUSS{ 1.0e-4 };

    Data data;
    data.linear_acceleration = Vector{ -MS2_PER_G * lin[1],
                                       -MS2_PER_G * lin[0],
                                       MS2_PER_G * lin[2] };
    data.angular_velocity = Vector{ RADS_PER_DEG * ang[1],
                                    RADS_PER_DEG * ang[0],
                                    -RADS_PER_DEG * ang[2] };
    data.magnetic_field = Vector{ TESLA_PER_GAUSS * mag[1],
                                  TESLA_PER_GAUSS * mag[0],
                                  -TESLA_PER_GAUSS * mag[2] };

    const std::chrono::duration<double, std::milli> since_epoch{ stamp };
    data.timestamp = Clock::time_point{ std::chrono::duration_cast<Clock::duration>(since_epoch) };

    return data;
}

DataIntervalReference& DataIntervalReference::operator=(Clock::duration data_interval) {
    spatial_.handle_ptr_->set_data_interval(data_interval);

    return *this;
}

DataIntervalReference& DataIntervalReference::operator=(
    const DataIntervalReference &data_interval
) {
    spatial_.handle_ptr_->set_data_interval(
        data_interval.spatial_.handle_ptr_->get_data_interval()
    );

    return *this;
}

DataIntervalReference::operator Clock::duration() const {
    return spatial_.handle_ptr_->get_data_interval();
}

DataIntervalReference::DataIntervalReference(Spatial spatial) noexcept
: spatial_{ std::move(spatial) } { }

} // namespace spatial
} // namespace ph
