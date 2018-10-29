#ifndef UMIGV_INVOKE_HPP
#define UMIGV_INVOKE_HPP

// backports std::invoke from C++17 to C++14

#include "umigv_utilities/traits.hpp"
#include "umigv_utilities/detail/invoke.hpp"

#include <functional>
#include <tuple>
#include <type_traits>
#include <utility>

namespace umigv {

template <typename T, typename ...As>
using invoke_result = detail::invoke_result<T, void, As...>;

template <typename T, typename ...As>
using invoke_result_t = typename invoke_result<T, As...>::type;

template <typename T, typename ...As>
using is_invocable = detail::is_invocable<T, void, As...>;

template <typename T, typename ...As>
constexpr bool is_invocable_v = is_invocable<T, As...>::value;

template <typename T, typename ...As>
using is_nothrow_invocable = detail::is_nothrow_invocable<T, void, As...>;

template <typename T, typename ...As>
constexpr bool is_nothrow_invocable_v = is_nothrow_invocable<T, As...>::value;

template <typename C, typename ...As,
          std::enable_if_t<is_invocable_v<C, As...>, int> = 0>
constexpr invoke_result_t<C, As...> invoke(C &&c, As &&...args)
noexcept(is_nothrow_invocable_v<C, As...>) {
    using TraitsT = typename detail::invoke_traits<C, void, As...>;
    using TypeT = typename TraitsT::type;

    return detail::invoke(TypeT{ }, std::forward<C>(c),
                          std::forward<As>(args)...);
}

} // namespace umigv

#endif
