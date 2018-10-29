#ifndef UMIGV_POINT2_HPP
#define UMIGV_POINT2_HPP

#include <cmath>
#include <cstddef>
#include <algorithm>
#include <functional>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <umigv_utilities/invoke.hpp>

namespace umigv {

struct Point2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Point2() noexcept = default;

    constexpr Point2(const double x_val, const double y_val) noexcept
    : x{ x_val }, y{ y_val } { }

    static constexpr Point2
    from_geometry(const geometry_msgs::Point &point) noexcept {
        return { point.x, point.y };
    }

    constexpr Point2& operator+=(const Point2 &other) noexcept {
        x += other.x;
        y += other.y;

        return *this;
    }

    constexpr Point2& operator-=(const Point2 &other) noexcept {
        x -= other.x;
        y -= other.y;

        return *this;
    }

    constexpr Point2& operator*=(const double z) noexcept {
        x *= z;
        y *= z;

        return *this;
    }

    constexpr Point2& operator/=(const double z) noexcept {
        x /= z;
        y /= z;

        return *this;
    }
};

constexpr bool operator==(const Point2 &lhs, const Point2 &rhs) noexcept {

    return lhs.x == rhs.x && lhs.y == rhs.y;
}

constexpr bool operator!=(const Point2 &lhs, const Point2 &rhs) noexcept {

    return !(lhs == rhs);
}

constexpr bool operator<(const Point2 &lhs, const Point2 &rhs) noexcept {
    if (lhs.x == rhs.x) {
        return lhs.y < rhs.y;
    }

    return lhs.x < rhs.x;
}

constexpr bool operator<=(const Point2 &lhs, const Point2 &rhs) noexcept {
    return !(rhs < lhs);
}

constexpr bool operator>(const Point2 &lhs, const Point2 &rhs) noexcept {
    return rhs < lhs;
}

constexpr bool operator>=(const Point2 &lhs, const Point2 &rhs) noexcept {
    return !(lhs < rhs);
}

constexpr Point2 operator+(const Point2 &lhs, const Point2 &rhs) noexcept {
    Point2 copy = lhs;

    return copy += rhs;
}

constexpr Point2 operator-(const Point2 &lhs, const Point2 &rhs) noexcept {
    Point2 copy = lhs;

    return copy -= rhs;
}

constexpr Point2 operator*(const Point2 &lhs, const double rhs) noexcept {
    Point2 copy = lhs;

    return copy *= rhs;
}

constexpr Point2 operator*(const double lhs, const Point2 &rhs) noexcept {
    Point2 copy = rhs;

    return copy *= lhs;
}

constexpr Point2 operator/(const Point2 &lhs, const double rhs) noexcept {
    Point2 copy = lhs;

    return copy /= rhs;
}

inline double abs(const Point2 &point) noexcept {
    return std::sqrt(point.x * point.x + point.y * point.y);
}

template <typename F,
         std::enable_if_t<is_invocable_v<F, const double&>, int> = 0>
constexpr Point2 apply(F &&f, const Point2 &point) noexcept {
    return { invoke(std::forward<F>(f), point.x),
             invoke(std::forward<F>(f), point.y) };
}

constexpr inline Point2 min(const Point2 &lhs, const Point2 &rhs) noexcept {
    return { std::min(lhs.x, rhs.x), std::min(lhs.y, rhs.y) };
}

constexpr inline Point2 max(const Point2 &lhs, const Point2 &rhs) noexcept {
    return { std::max(lhs.x, rhs.x), std::max(lhs.y, rhs.y) };
}

template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Point2 &point) {
    return os << '[' << point.x << ' ' << point.y << ']';
}

} // namespace umigv

namespace std {

template <>
struct hash<umigv::Point2> {
    inline std::size_t operator()(const umigv::Point2 &point) const noexcept {
        const std::hash<double> hasher{ };

        const std::size_t x_hash = hasher(point.x);
        const std::size_t y_hash = hasher(point.y);

        return x_hash ^ y_hash + 0x9e3779b9 + (x_hash << 6) + (x_hash >> 2);
    }
};

} // namespace std

#endif
