#ifndef UMIGV_UTILITY_HPP
#define UMIGV_UTILITY_HPP

// utility header; useful but otherwise ungrouped functions

#include "umigv_utilities/ros.hpp"
#include "umigv_utilities/traits.hpp"

#include <iterator>
#include <type_traits>
#include <utility>

namespace umigv {

// converts to To from From via type punning
template <typename To, typename From,
          typename = std::enable_if_t<sizeof(To) == sizeof(From) &&
                                      alignof(To) == alignof(From)>>
constexpr To& byte_cast(From &from) noexcept {
    return *reinterpret_cast<To*>(&from);
}

// converts to To from From via type punning
template <typename To, typename From,
          typename = std::enable_if_t<sizeof(To) == sizeof(From) &&
                                      alignof(To) == alignof(From)>>
constexpr const To& byte_cast(const From &from) noexcept {
    return *reinterpret_cast<const To*>(&from);
}

inline namespace adl {

using std::swap;
using std::begin;
using std::end;

template <typename T, std::enable_if_t<umigv::has_begin_v<T>, int> = 0>
constexpr umigv::begin_result_t<T> adl_begin(T &&t)
noexcept(umigv::has_nothrow_begin_v<T>) {
    return begin(std::forward<T>(t));
}

template <typename T, std::enable_if_t<umigv::has_end_v<T>, int> = 0>
constexpr umigv::end_result_t<T> adl_end(T &&t)
noexcept(umigv::has_nothrow_end_v<T>) {
    return end(std::forward<T>(t));
}

template <typename T, typename U,
          std::enable_if_t<umigv::is_swappable_with_v<T, U>, int> = 0>
constexpr void adl_swap(T &&t, U &&u)
noexcept(umigv::is_nothrow_swappable_with_v<T, U>) {
    return swap(std::forward<T>(t), std::forward<U>(u));
}

} // namespace adl
} // namespace umigv

#endif
