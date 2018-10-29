#ifndef UMIGV_ROS_HPP
#define UMIGV_ROS_HPP

// utilities relating to ROS

#include "umigv_utilities/exceptions.hpp"
#include "umigv_utilities/invoke.hpp"
#include "umigv_utilities/traits.hpp"

#include <cstdlib>
#include <string>
#include <type_traits>
#include <utility>

#include <boost/optional.hpp>

#include <ros/ros.h>

namespace umigv {

// fetches parameter from rosparam using handle
// throws umigv::ParameterNotFoundException if
// unable to fetch from parameter server
template <typename T, std::enable_if_t<is_rosparam_v<T>, int> = 0>
T get_parameter_fatal(const ros::NodeHandle &handle,
                      const std::string &parameter) {
    T fetched;

    if (not handle.getParam(parameter, fetched)) {
        throw ParameterNotFoundException{ "get_parameter_fatal", parameter };
    }

    return fetched;
}

// fetches parameter from rosparam using handle
// returns default value if unable to fetch
template <typename T, std::enable_if_t<is_rosparam_v<T>, int> = 0>
T get_parameter_or(const ros::NodeHandle &handle, const std::string &name,
                   const T &fallback = T{ }) {
    T fetched;

    handle.param(name, fetched, fallback);

    return fetched;
}

[[noreturn]] inline void blocking_shutdown() noexcept {
    ros::shutdown();
    ros::waitForShutdown();
    std::exit(EXIT_FAILURE);
}

class ParameterServer;

class ParameterReference {
public:
    friend ParameterServer;

    template <typename T, std::enable_if_t<is_rosparam_v<T>, int> = 0>
    boost::optional<T> value() const;

    template <typename T, std::enable_if_t<is_rosparam_v<T>, int> = 0>
    T value_or(const T &fallback = T{ });

    template <
        typename T, typename C,
        std::enable_if_t<
            is_rosparam_v<T> && is_invocable_v<C>
            && std::is_convertible<invoke_result_t<C>, T>::value, int
        > = 0
    >
    T value_or_eval(C &&callable);

    template <typename T, std::enable_if_t<is_rosparam_v<T>, int> = 0>
    T value_or_throw();

private:
    inline ParameterReference(const ParameterServer &parent,
                              std::string key) noexcept;

    const ParameterServer *parent_ptr_ = nullptr;
    std::string key_;
};

class ParameterServer {
public:
    template <
        typename ...Ts,
        std::enable_if_t<std::is_constructible<ros::NodeHandle, Ts...>::value,
                         int> = 0
    >
    ParameterServer(const bool should_cache, Ts &&...args)
    noexcept(std::is_nothrow_constructible<ros::NodeHandle, Ts...>::value)
    : node_(std::forward<Ts>(args)...), should_cache_{ should_cache } { }

    void enable_caching() noexcept {
        should_cache_ = true;
    }

    void disable_caching() noexcept {
        should_cache_ = false;
    }

    bool has_parameter(const std::string &key) const {
        return node_.hasParam(key);
    }

    boost::optional<std::vector<std::string>> get_names() const {
        std::vector<std::string> keys;

        if (!node_.getParamNames(keys)) {
            return boost::none;
        }

        return { std::move(keys) };
    }

    template <typename T, std::enable_if_t<is_rosparam_v<T>, int> = 0>
    boost::optional<T> get(const std::string &key) const {
        T parameter;

        const bool was_fetched = [this, &key, &parameter]() mutable {
            if (should_cache_) {
                return node_.getParamCached(key, parameter);
            }

            return node_.getParam(key, parameter);
        }();

        if (!was_fetched) {
            return boost::none;
        }

        return { std::move(parameter) };
    }

    ParameterReference operator[](std::string key) const {
        return { *this, std::move(key) };
    }

private:
    ros::NodeHandle node_;
    bool should_cache_ = false;
};

template <typename T, std::enable_if_t<is_rosparam_v<T>, int>>
boost::optional<T> ParameterReference::value() const {
    return parent_ptr_->get<T>(key_);
}

template <typename T, std::enable_if_t<is_rosparam_v<T>, int>>
T ParameterReference::value_or(const T &fallback) {
    const boost::optional<T> maybe_value = value<T>();

    if (maybe_value) {
        // no NRVO with subobjects
        return std::move(maybe_value.value());
    }

    return fallback;
}

template <
    typename T, typename C,
    std::enable_if_t<
        is_rosparam_v<T> && is_invocable_v<C>
        && std::is_convertible<invoke_result_t<C>, T>::value, int
    >
>
T ParameterReference::value_or_eval(C &&callable) {
    const boost::optional<T> maybe_value = value<T>();

    if (maybe_value) {
        // no NRVO with subobjects
        return std::move(maybe_value.value());
    }

    return invoke(std::forward<C>(callable));
}

template <typename T, std::enable_if_t<is_rosparam_v<T>, int>>
T ParameterReference::value_or_throw() {
    const boost::optional<T> maybe_value = value<T>();

    if (maybe_value) {
        return std::move(maybe_value.value());
    }

    throw ParameterNotFoundException{ "ParameterReference::value_or_throw",
                                      key_ };
}

inline ParameterReference::ParameterReference(const ParameterServer &parent,
                                              std::string key) noexcept
: parent_ptr_{ &parent }, key_{ std::move(key) } { }

} // namespace umigv

#endif
