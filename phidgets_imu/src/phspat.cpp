#include <phspat.h>

#include <cassert>
#include <functional>
#include <mutex>
#include <string>
#include <system_error>
#include <utility>

namespace ph {

static void expect(PhidgetReturnCode ret, const char *what) {
    assert(what);

    if (ret == EPHIDGET_OK) {
        return;
    }

    throw std::system_error{ static_cast<int>(ret), phidget_category(), what };
}

class PhidgetCategory : public std::error_category {
public:
    virtual ~PhidgetCategory() = default;

    const char* name() const noexcept override {
        return "ph::PhidgetCategory";
    }

    std::string message(int condition) const override {
        const char *msg;
        const auto ret = Phidget_getErrorDescription(static_cast<PhidgetReturnCode>(condition), &msg);

        if (ret != EPHIDGET_OK) {
            std::abort();
        }

        return { msg };
    }
};

const std::error_category& phidget_category() noexcept {
    static const PhidgetCategory category;

    return category;
}

namespace spatial {

class Handle : public std::enable_shared_from_this<Handle> {
public:
    using AttachHandler = Spatial::AttachHandler;
    using DetachHandler = Spatial::DetachHandler;
    using DataHandler = Spatial::DataHandler;

    Handle(const spatial::Config &config) {
        const auto expect = [](PhidgetReturnCode ret) {
            ph::expect(ret, "ph::spatial::Handle::Handle()");
        };

        expect(PhidgetSpatial_create(&base_));
        expect(Phidget_setChannel(as_phidget(), config.channel));
        expect(Phidget_setDeviceSerialNumber(as_phidget(), config.serial_number));

        if (config.label) {
            expect(Phidget_setDeviceLabel(as_phidget(), config.label->c_str()));
        } else {
            expect(Phidget_setDeviceLabel(as_phidget(), PHIDGET_LABEL_ANY));
        }

        expect(Phidget_setOnAttachHandler(as_phidget(), &Handle::handle_attach, this));
        expect(Phidget_setOnDetachHandler(as_phidget(), &Handle::handle_detach, this));
        expect(PhidgetSpatial_setOnSpatialDataHandler(base_, &Handle::handle_data, this));

        expect(Phidget_open(as_phidget()));
    }

    ~Handle() {
        const auto expect = [](PhidgetReturnCode ret) {
            ph::expect(ret, "ph::spatial::Handle::~Handle()");
        };

        expect(Phidget_close(as_phidget()));
        expect(PhidgetSpatial_delete(&base_));
    }

    void set_attach_handler(std::function<AttachHandler> f) {
        const std::lock_guard<std::recursive_mutex> lck{ mtx_ };

        attach_handler_ = std::move(f);
    }

    void set_detach_handler(std::function<DetachHandler> f) {
        const std::lock_guard<std::recursive_mutex> lck{ mtx_ };

        detach_handler_ = std::move(f);
    }

    void set_data_handler(std::function<DataHandler> f) {
        const std::lock_guard<std::recursive_mutex> lck{ mtx_ };

        data_handler_ = std::move(f);
    }

    std::uint32_t get_min_data_interval() const {
        const std::lock_guard<std::recursive_mutex> lck{ mtx_ };

        std::uint32_t min_data_interval;
        expect(PhidgetSpatial_getMinDataInterval(base_, &min_data_interval),
               "ph::spatial::Handle::get_min_data_interval");

        return min_data_interval;
    }

    std::uint32_t get_max_data_interval() const {
        const std::lock_guard<std::recursive_mutex> lck{ mtx_ };

        std::uint32_t max_data_interval;
        expect(PhidgetSpatial_getMaxDataInterval(base_, &max_data_interval),
               "ph::spatial::Handle::get_max_data_interval");

        return max_data_interval;
    }

    std::uint32_t get_data_interval() const {
        const std::lock_guard<std::recursive_mutex> lck{ mtx_ };

        std::uint32_t data_interval;
        expect(PhidgetSpatial_getDataInterval(base_, &data_interval),
               "ph::spatial::Handle::get_data_interval");

        return data_interval;
    }

    void set_data_interval(std::uint32_t data_interval) {
        const std::lock_guard<std::recursive_mutex> lck{ mtx_ };

        expect(PhidgetSpatial_setDataInterval(base_, data_interval),
               "ph::spatial::Handle::set_data_interval");
    }

private:
    static void handle_attach(PhidgetHandle, void *handle_v) {
        auto &handle{ *static_cast<Handle*>(handle_v) };

        const std::lock_guard<std::recursive_mutex> lck{ handle.mtx_ };

        if (handle.attach_handler_) {
            Spatial spatial{ handle.shared_from_this() };
            handle.attach_handler_(spatial);
        }
    }

    static void handle_detach(PhidgetHandle, void *handle_v) {
        auto &handle{ *static_cast<Handle*>(handle_v) };

        const std::lock_guard<std::recursive_mutex> lck{ handle.mtx_ };

        if (handle.detach_handler_) {
            Spatial spatial{ handle.shared_from_this() };
            handle.detach_handler_(spatial);
        }
    }

    static void handle_data(PhidgetSpatialHandle, void *handle_v, const double acceleration[3],
                            const double angular_rate[3], const double magnetic_field[3],
                            double timestamp) {
        constexpr double MS2_PER_G{ 9.80665 };
        constexpr double RADS_PER_DEG{ 0.017453292519943295769 };
        constexpr double TESLA_PER_GAUSS{ 1.0e-4 };

        auto &handle{ *static_cast<Handle*>(handle_v) };

        const std::lock_guard<std::recursive_mutex> lck{ handle.mtx_ };

        // imu coordinate frame is NED by default - transform to ENU
        // ready for use with ROS - follows REP 103, REP 145, robot_localization standards
        // also transform units to metric standard
        if (handle.data_handler_) {
            const std::array<double, 3> linear{ -MS2_PER_G * acceleration[1],
                                                -MS2_PER_G * acceleration[0],
                                                MS2_PER_G * acceleration[2] };
            const std::array<double, 3> angular{ -RADS_PER_DEG * angular_rate[1],
                                                 -RADS_PER_DEG * angular_rate[0],
                                                 RADS_PER_DEG * angular_rate[2] };
            const std::array<double, 3> magnetic{ -TESLA_PER_GAUSS * magnetic_field[1],
                                                  -TESLA_PER_GAUSS * magnetic_field[0],
                                                  TESLA_PER_GAUSS * magnetic_field[2] };

            Spatial spatial{ handle.shared_from_this() };
            handle.data_handler_(spatial, linear, angular, magnetic, timestamp);
        }
    }

    PhidgetHandle as_phidget() const noexcept {
        return reinterpret_cast<PhidgetHandle>(base_);
    }

    PhidgetSpatialHandle base_;
    std::function<AttachHandler> attach_handler_;
    std::function<DetachHandler> detach_handler_;
    std::function<DataHandler> data_handler_;
    mutable std::recursive_mutex mtx_;
};

} // namespace spatial

Spatial::Spatial() : Spatial{ spatial::Config{ } } { }

Spatial::Spatial(const spatial::Config &config)
: Spatial{ std::make_unique<spatial::Handle>(config) } { }

spatial::DataIntervalReference Spatial::data_interval() noexcept {
    return spatial::DataIntervalReference{ *this };
}

std::uint32_t Spatial::data_interval() const {
    return handle_ptr_->get_data_interval();
}

std::uint32_t Spatial::min_data_interval() const {
    return handle_ptr_->get_min_data_interval();
}

std::uint32_t Spatial::max_data_interval() const {
    return handle_ptr_->get_max_data_interval();
}

void Spatial::set_attach_handler(std::function<AttachHandler> f) {
    handle_ptr_->set_attach_handler(std::move(f));
}

void Spatial::set_detach_handler(std::function<DetachHandler> f) {
    handle_ptr_->set_detach_handler(std::move(f));
}

void Spatial::set_data_handler(std::function<DataHandler> f) {
    handle_ptr_->set_data_handler(std::move(f));
}

Spatial::Spatial(std::shared_ptr<spatial::Handle> handle_ptr) noexcept
: handle_ptr_{ std::move(handle_ptr) } {
    assert(handle_ptr_);
}

namespace spatial {

DataIntervalReference& DataIntervalReference::operator=(std::uint32_t data_interval) {
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

DataIntervalReference::operator std::uint32_t() const {
    return spatial_.handle_ptr_->get_data_interval();
}

DataIntervalReference::DataIntervalReference(Spatial spatial) noexcept
: spatial_{ std::move(spatial) } { }

} // namespace spatial
} // namespace ph
