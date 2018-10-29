#ifndef UMIGV_DETAIL_INVOKE_TRAITS_HPP
#define UMIGV_DETAIL_INVOKE_TRAITS_HPP

#include "umigv_utilities/traits.hpp"

#include <type_traits>
#include <utility>

namespace umigv {
namespace detail {

struct member_function_ref_tag { };

struct member_function_ptr_tag { };

struct member_data_ref_tag { };

struct member_data_ptr_tag { };

struct functor_tag { };

template <typename T, typename, typename ...Args>
struct invoke_traits { };

template <typename T, typename U, typename V, typename ...Args>
struct invoke_traits<
    T U::*,
    void_t<decltype(
        (std::declval<V>().*std::declval<T U::*>())(std::declval<Args>()...)
    ), std::enable_if_t<std::is_member_function_pointer<T U::*>::value>,
    std::enable_if_t<std::is_base_of<U, std::decay_t<V>>::value>>,
    V, Args...
> {
    using type = member_function_ref_tag;
    using result = decltype((
        (std::declval<V>().*std::declval<T U::*>())(std::declval<Args>()...)
    ));

    static constexpr bool is_nothrow = noexcept(
        (std::declval<V>().*std::declval<T U::*>())(std::declval<Args>()...)
    );
};

template <typename T, typename U, typename ...Args>
struct invoke_traits<
    T,
    void_t<decltype(
        ((*std::declval<U>()).*std::declval<T>())(
            std::declval<Args>()...
        )
    ), std::enable_if_t<std::is_member_function_pointer<T>::value>>,
    U, Args...
> {
    using type = member_function_ptr_tag;
    using result = decltype((
        ((*std::declval<U>()).*std::declval<T>())(
            std::declval<Args>()...
        )
    ));

    static constexpr bool is_nothrow = noexcept(
        ((*std::declval<U>()).*std::declval<T>())(
            std::declval<Args>()...
        )
    );
};

template <typename T, typename U, typename V>
struct invoke_traits<
    T U::*,
    void_t<decltype(
        std::declval<V>().*std::declval<T U::*>()
    ), std::enable_if_t<std::is_member_object_pointer<T U::*>::value>,
    std::enable_if_t<std::is_base_of<U, std::decay_t<V>>::value>>,
    V
> {
    using type = member_data_ref_tag;
    using result = decltype((
        std::declval<V>().*std::declval<T U::*>()
    ));

    static constexpr bool is_nothrow = noexcept(
        std::declval<V>().*std::declval<T U::*>()
    );
};

template <typename T, typename U>
struct invoke_traits<
    T,
    void_t<decltype(
        (*std::declval<U>()).*std::declval<T>()
    ), std::enable_if_t<std::is_member_object_pointer<T>::value>>,
    U
> {
    using type = member_data_ptr_tag;
    using result = decltype((
        (*std::declval<U>()).*std::declval<T>()
    ));

    static constexpr bool is_nothrow = noexcept(
        (*std::declval<U>()).*std::declval<T>()
    );
};

template <typename T, typename ...Args>
struct invoke_traits<
    T,
    void_t<decltype(
        std::declval<T>()(std::declval<Args>()...)
    )>,
    Args...
> {
    using type = functor_tag;
    using result = decltype((
        std::declval<T>()(std::declval<Args>()...)
    ));

    static constexpr bool is_nothrow = noexcept(
        std::declval<T>()(std::declval<Args>()...)
    );
};

} // namespace detail
} // namespace umigv

#endif
