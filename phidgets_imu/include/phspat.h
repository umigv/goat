#ifndef PHSPAT_H
#define PHSPAT_H

#include <cstdint>
#include <array>
#include <memory>
#include <system_error>

#include <boost/optional.hpp>

#include <phidget22.h>

namespace ph {

class Spatial;

namespace spatial {

class Handle;

struct Config {
    int channel{ PHIDGET_CHANNEL_ANY };
    std::int32_t serial_number{ PHIDGET_SERIALNUMBER_ANY };
    boost::optional<std::string> label;
};

class DataIntervalReference;

} // namespace spatial

class Spatial {
public:
    friend spatial::DataIntervalReference;
    friend spatial::Handle;

    using AttachHandler = void(Spatial&);
    using DetachHandler = void(Spatial&);
    using DataHandler = void(Spatial&, const std::array<double, 3>, const std::array<double, 3>,
                             const std::array<double, 3>, double);

    Spatial();

    explicit Spatial(const spatial::Config &config);

    Spatial(const Spatial &other) = default;

    Spatial& operator=(const Spatial &other) = default;

    spatial::DataIntervalReference data_interval() noexcept;

    std::uint32_t data_interval() const;

    std::uint32_t min_data_interval() const;

    std::uint32_t max_data_interval() const;

    void set_attach_handler(std::function<AttachHandler> f);

    void set_detach_handler(std::function<DetachHandler> f);

    void set_data_handler(std::function<DataHandler> f);

private:
    explicit Spatial(std::shared_ptr<spatial::Handle> handle_ptr) noexcept;

    Spatial(Spatial &&other) = default;

    Spatial& operator=(Spatial &&other) = default;

    std::shared_ptr<spatial::Handle> handle_ptr_;
};

const std::error_category& phidget_category() noexcept;

namespace spatial {

class DataIntervalReference {
public:
    friend Spatial;

    DataIntervalReference(const DataIntervalReference &other) = default;

    DataIntervalReference& operator=(std::uint32_t data_interval);

    DataIntervalReference& operator=(const DataIntervalReference &data_interval);

    operator std::uint32_t() const;

private:
    explicit DataIntervalReference(Spatial spatial) noexcept;

    Spatial spatial_;
};

} // namespace spatial
} // namespace ph

#endif
