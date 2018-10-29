#ifndef UMIGV_DETAIL_INVOKE_HPP
#define UMIGV_DETAIL_INVOKE_HPP

#include "umigv_utilities/traits.hpp"
#include "umigv_utilities/detail/invoke_traits.hpp"

#include <type_traits>

namespace umigv {
namespace detail {

template <typename T, typename, typename ...As>
struct invoke_result { };

template <typename T, typename ...As>
struct invoke_result<
    T, void_t<typename invoke_traits<T, void, As...>::result>, As...
> {
    using type = typename invoke_traits<T, void, As...>::result;
};

template <typename T, typename, typename ...As>
struct is_invocable : std::false_type { };

template <typename T, typename ...As>
struct is_invocable<
    T, void_t<typename invoke_traits<T, void, As...>::result>, As...
> : std::true_type { };

template <typename T, typename, typename ...As>
struct is_nothrow_invocable : std::false_type { };

template <typename T, typename ...As>
struct is_nothrow_invocable<
    T, void_t<
        typename invoke_traits<T, void, As...>::result,
        std::enable_if_t<invoke_traits<T, void, As...>::is_nothrow>
    >, As...
> : std::true_type { };

template <typename T, typename U, typename ...As>
constexpr decltype(auto) invoke(member_function_ref_tag, T &&t, U &&u,
                                As &&...args)
noexcept(is_nothrow_invocable<T, void, U, As...>::value) {
    return (std::forward<U>(u).*std::forward<T>(t))(std::forward<As>(args)...);
}

template <typename T, typename U, typename ...As>
constexpr decltype(auto) invoke(member_function_ptr_tag, T &&t, U &&u,
                                As &&...args)
noexcept(is_nothrow_invocable<T, void, U, As...>::value) {
    return ((*std::forward<U>(u)).*std::forward<T>(t))(
        std::forward<As>(args)...
    );
}

template <typename T, typename U>
constexpr decltype(auto) invoke(member_data_ref_tag, T &&t, U &&u)
noexcept(is_nothrow_invocable<T, void, U>::value) {
    return std::forward<U>(u).*std::forward<T>(t);
}

template <typename T, typename U>
constexpr decltype(auto) invoke(member_data_ptr_tag, T &&t, U &&u)
noexcept(is_nothrow_invocable<T, void, U>::value) {
    return (*std::forward<U>(u)).*std::forward<T>(t);
}

template <typename T, typename ...As>
constexpr decltype(auto) invoke(functor_tag, T &&t, As &&...args)
noexcept(is_nothrow_invocable<T, void, As...>::value) {
    return std::forward<T>(t)(std::forward<As>(args)...);
}

} // namespace detail
} // namespace umigv

#endif
