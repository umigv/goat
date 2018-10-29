#ifndef UMIGV_TYPES_HPP
#define UMIGV_TYPES_HPP

// type support header; defines a few common typedefs and user-defined literals

#include <cstddef>
#include <cstdint>

namespace umigv {
inline namespace types {

using i8 = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using isize = std::ptrdiff_t;
using usize = std::size_t;

using f32 = float;
using f64 = double;

} // inline namespace types

inline namespace literals {

constexpr i8 operator""_i8(const unsigned long long literal) noexcept {
    return static_cast<i8>(literal);
}

constexpr i16 operator""_i16(const unsigned long long literal) noexcept {
    return static_cast<i16>(literal);
}

constexpr i32 operator""_i32(const unsigned long long literal) noexcept {
    return static_cast<i32>(literal);
}

constexpr i64 operator""_i64(const unsigned long long literal) noexcept {
    return static_cast<i64>(literal);
}

constexpr u8 operator""_u8(const unsigned long long literal) noexcept {
    return static_cast<u8>(literal);
}

constexpr u16 operator""_u16(const unsigned long long literal) noexcept {
    return static_cast<u16>(literal);
}

constexpr u32 operator""_u32(const unsigned long long literal) noexcept {
    return static_cast<u32>(literal);
}

constexpr u64 operator""_u64(const unsigned long long literal) noexcept {
    return static_cast<u64>(literal);
}

constexpr isize operator""_isize(const unsigned long long literal) noexcept {
    return static_cast<isize>(literal);
}

constexpr usize operator""_usize(const unsigned long long literal) noexcept {
    return static_cast<usize>(literal);
}

constexpr f32 operator""_f32(const long double literal) noexcept {
    return static_cast<f32>(literal);
}

constexpr f64 operator""_f64(const long double literal) noexcept {
    return static_cast<f64>(literal);
}

} // inline namespace literals
} // namespace umigv

#endif
