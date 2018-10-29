#ifndef UMIGV_TRAITS_HPP
#define UMIGV_TRAITS_HPP

// type traits header; defines custom type traits

#include "umigv_utilities/types.hpp"

#include <iterator>
#include <map>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace umigv {

template <typename ...Ts>
using void_t = void;

template <bool Condition>
struct true_type_if : std::false_type { };

template <>
struct true_type_if<true> : std::true_type { };

template <bool Condition>
using true_type_if_t = typename true_type_if<Condition>::type;

template <typename T>
struct is_rosparam_literal : std::false_type { };

template <typename T>
constexpr bool is_rosparam_literal_v = is_rosparam_literal<T>::value;

template <>
struct is_rosparam_literal<bool> : std::true_type { };

template <>
struct is_rosparam_literal<int> : std::true_type { };

template <>
struct is_rosparam_literal<float> : std::true_type { };

template <>
struct is_rosparam_literal<double> : std::true_type { };

template <>
struct is_rosparam_literal<std::string> : std::true_type { };

template <typename T>
struct is_rosparam : is_rosparam_literal<T> { };

template <typename T>
constexpr bool is_rosparam_v = is_rosparam<T>::value;

template <typename T>
struct is_rosparam<std::vector<T>> : is_rosparam_literal<T> { };

template <typename T>
struct is_rosparam<std::map<std::string, T>> : is_rosparam_literal<T> { };

template <typename T, typename = void>
struct is_iterator : std::false_type { };

template <typename T>
struct is_iterator<
    T,
    void_t<typename std::iterator_traits<T>::difference_type,
           typename std::iterator_traits<T>::value_type,
           typename std::iterator_traits<T>::pointer,
           typename std::iterator_traits<T>::reference,
           typename std::iterator_traits<T>::iterator_category>
> : std::true_type { };

template <typename T>
constexpr bool is_iterator_v = is_iterator<T>::value;

template <typename T, bool IsIterator = is_iterator_v<T>>
struct iterator_difference_type { };

template <typename T>
struct iterator_difference_type<T, true> {
    using type = typename std::iterator_traits<T>::difference_type;
};

template <typename T>
using iterator_difference_type_t = typename iterator_difference_type<T>::type;

template <typename T, bool IsIterator = is_iterator_v<T>>
struct iterator_value_type { };

template <typename T>
struct iterator_value_type<T, true> {
    using type = typename std::iterator_traits<T>::value_type;
};

template <typename T>
using iterator_value_type_t = typename iterator_value_type<T>::type;

template <typename T, bool IsIterator = is_iterator_v<T>>
struct iterator_pointer { };

template <typename T>
struct iterator_pointer<T, true> {
    using type = typename std::iterator_traits<T>::pointer;
};

template <typename T>
using iterator_pointer_t = typename iterator_pointer<T>::type;

template <typename T, bool IsIterator = is_iterator_v<T>>
struct iterator_reference { };

template <typename T>
struct iterator_reference<T, true> {
    using type = typename std::iterator_traits<T>::reference;
};

template <typename T>
using iterator_reference_t = typename iterator_reference<T>::type;

template <typename T, bool IsIterator = is_iterator_v<T>>
struct iterator_category { };

template <typename T>
struct iterator_category<T, true> {
    using type = typename std::iterator_traits<T>::iterator_category;
};

template <typename T>
using iterator_category_t = typename iterator_category<T>::type;

template <typename T, bool IsIterator = is_iterator_v<T>>
struct is_input_iterator : std::false_type { };

template <typename T>
struct is_input_iterator<T, true>
: true_type_if_t<std::is_base_of<std::input_iterator_tag,
                                 iterator_category_t<T>>::value> { };

template <typename T>
constexpr bool is_input_iterator_v = is_input_iterator<T>::value;

template <typename T, bool IsIterator = is_iterator_v<T>>
struct is_output_iterator : std::false_type { };

template <typename T>
struct is_output_iterator<T, true>
: true_type_if_t<std::is_base_of<std::output_iterator_tag,
                                 iterator_category_t<T>>::value> { };

template <typename T>
constexpr bool is_output_iterator_v = is_output_iterator<T>::value;

template <typename T, bool IsIterator = is_iterator_v<T>>
struct is_forward_iterator : std::false_type { };

template <typename T>
struct is_forward_iterator<T, true>
: true_type_if_t<std::is_base_of<std::forward_iterator_tag,
                                 iterator_category_t<T>>::value> { };

template <typename T>
constexpr bool is_forward_iterator_v = is_forward_iterator<T>::value;

template <typename T, bool IsIterator = is_iterator_v<T>>
struct is_bidirectional_iterator : std::false_type { };

template <typename T>
struct is_bidirectional_iterator<T, true>
: true_type_if_t<std::is_base_of<std::bidirectional_iterator_tag,
                                 iterator_category_t<T>>::value> { };

template <typename T>
constexpr bool is_bidirectional_iterator_v
    = is_bidirectional_iterator<T>::value;

template <typename T, bool IsIterator = is_iterator_v<T>>
struct is_random_access_iterator : std::false_type { };

template <typename T>
struct is_random_access_iterator<T, true>
: true_type_if_t<std::is_base_of<std::random_access_iterator_tag,
                                 iterator_category_t<T>>::value> { };

template <typename T>
constexpr bool is_random_access_iterator_v
    = is_random_access_iterator<T>::value;

} // namespace umigv

// separate namespace to avoid lookup issues
namespace umigv_detail_adl {

using std::swap;
using std::begin;
using std::end;

template <typename T, typename U, typename = void>
struct is_swappable_with : std::false_type { };

template <typename T, typename U>
struct is_swappable_with<
    T, U,
    umigv::void_t<decltype(swap(std::declval<T>(), std::declval<U>()))>
> : std::true_type { };

template <typename T>
struct is_swappable : is_swappable_with<T, T> { };

template <typename T, typename U,
          bool IsSwappable = is_swappable_with<T, U>::value>
struct is_nothrow_swappable_with : std::false_type { };

template <typename T, typename U>
struct is_nothrow_swappable_with<T, U, true>
: umigv::true_type_if_t<noexcept(swap(std::declval<T>(),
                                      std::declval<U>()))> { };

template <typename T>
struct is_nothrow_swappable : is_nothrow_swappable_with<T, T> { };

template <typename T, typename = void>
struct has_begin : std::false_type { };

template <typename T>
struct has_begin<T, umigv::void_t<decltype(begin(std::declval<T>()))>>
: std::true_type { };

template <typename T, typename = void>
struct has_end : std::false_type { };

template <typename T>
struct has_end<T, umigv::void_t<decltype(end(std::declval<T>()))>>
: std::true_type { };

template <typename T, bool = has_begin<T>::value>
struct has_nothrow_begin : std::false_type { };

template <typename T>
struct has_nothrow_begin<T, true>
: umigv::true_type_if_t<noexcept(begin(std::declval<T>()))> { };

template <typename T, bool = has_end<T>::value>
struct has_nothrow_end : std::false_type { };

template <typename T>
struct has_nothrow_end<T, true>
: umigv::true_type_if_t<noexcept(end(std::declval<T>()))> { };

template <typename T, bool = has_begin<T>::value>
struct begin_result { };

template <typename T>
struct begin_result<T, true> {
    using type = decltype(begin(std::declval<T>()));
};

template <typename T>
using begin_result_t = typename begin_result<T>::type;

template <typename T, bool = has_end<T>::value>
struct end_result { };

template <typename T>
struct end_result<T, true> {
    using type = decltype(end(std::declval<T>()));
};

template <typename T>
using end_result_t = typename end_result<T>::type;

} // namespace umigv_detail_adl

namespace umigv {

template <typename T, typename U>
struct is_swappable_with : umigv_detail_adl::is_swappable_with<T, U> { };

template <typename T, typename U>
constexpr bool is_swappable_with_v = is_swappable_with<T, U>::value;

template <typename T>
struct is_swappable : umigv_detail_adl::is_swappable<T> { };

template <typename T>
constexpr bool is_swappable_v = is_swappable<T>::value;

template <typename T, typename U>
struct is_nothrow_swappable_with
: umigv_detail_adl::is_nothrow_swappable_with<T, U> { };

template <typename T, typename U>
constexpr bool is_nothrow_swappable_with_v =
    is_nothrow_swappable_with<T, U>::value;

template <typename T>
struct is_nothrow_swappable : umigv_detail_adl::is_nothrow_swappable<T> { };

template <typename T>
constexpr bool is_nothrow_swappable_v = is_nothrow_swappable<T>::value;

template <typename T>
struct has_begin : umigv_detail_adl::has_begin<T> { };

template <typename T>
constexpr bool has_begin_v = has_begin<T>::value;

template <typename T>
struct has_end : umigv_detail_adl::has_end<T> { };

template <typename T>
constexpr bool has_end_v = has_end<T>::value;

template <typename T>
struct has_nothrow_begin : umigv_detail_adl::has_nothrow_begin<T> { };

template <typename T>
constexpr bool has_nothrow_begin_v = has_nothrow_begin<T>::value;

template <typename T>
struct has_nothrow_end : umigv_detail_adl::has_nothrow_end<T> { };

template <typename T>
constexpr bool has_nothrow_end_v = has_nothrow_end<T>::value;

template <typename T>
struct begin_result : umigv_detail_adl::begin_result<T> { };

template <typename T>
using begin_result_t = typename begin_result<T>::type;

template <typename T>
struct end_result : umigv_detail_adl::end_result<T> { };

template <typename T>
using end_result_t = typename end_result<T>::type;

template <typename T, typename = void>
          // bool = has_begin_v<T> && has_end_v<T>>
struct is_range : std::false_type { };

template <typename T>
struct is_range<
    T,
    void_t<std::enable_if_t<
        has_begin_v<T> && has_end_v<T>
        && std::is_same<begin_result_t<T>, end_result_t<T>>::value
        && is_iterator_v<begin_result_t<T>>
    >>
> : std::true_type { };

template <typename T>
constexpr bool is_range_v = is_range<T>::value;

template <typename T, bool = is_range_v<T>>
struct range_value_type { };

template <typename T>
struct range_value_type<T, true> {
    using type = iterator_value_type_t<begin_result_t<T>>;
};

template <typename T>
using range_value_type_t = typename range_value_type<T>::type;

template <typename T, bool = is_range_v<T>>
struct range_reference { };

template <typename T>
struct range_reference<T, true> {
    using type = iterator_reference_t<begin_result_t<T>>;
};

template <typename T>
using range_reference_t = typename range_reference<T>::type;

template <typename T, bool = is_range_v<T>>
struct range_difference_type { };

template <typename T>
struct range_difference_type<T, true> {
    using type = iterator_difference_type_t<begin_result_t<T>>;
};

template <typename T>
using range_difference_type_t = typename range_difference_type<T>::type;

template <typename T, bool = is_range_v<T>>
struct range_pointer { };

template <typename T>
struct range_pointer<T, true> {
    using type = iterator_pointer_t<begin_result_t<T>>;
};

template <typename T>
using range_pointer_t = typename range_pointer<T>::type;

template <typename T, bool = is_range_v<T>>
struct range_category { };

template <typename T>
struct range_category<T, true> {
    using type = iterator_category_t<begin_result_t<T>>;
};

template <typename T>
using range_category_t = typename range_category<T>::type;

template <typename T, typename U, typename = void>
struct is_equality_comparable_with : std::false_type { };

template <typename T, typename U>
struct is_equality_comparable_with<
    T, U, void_t<decltype(std::declval<T>() == std::declval<U>())>
> : std::true_type { };

template <typename T, typename U>
constexpr bool is_equality_comparable_with_v =
    is_equality_comparable_with<T, U>::value;

template <typename T>
struct is_equality_comparable : is_equality_comparable_with<T, T> { };

template <typename T>
constexpr bool is_equality_comparable_v =
    is_equality_comparable<T>::value;

template <typename T, typename U, bool = is_equality_comparable_with_v<T, U>>
struct is_nothrow_equality_comparable_with : std::false_type { };

template <typename T, typename U>
struct is_nothrow_equality_comparable_with<T, U, true>
: true_type_if_t<noexcept(std::declval<T>() == std::declval<U>())> { };

template <typename T, typename U>
constexpr bool is_nothrow_equality_comparable_with_v =
    is_nothrow_equality_comparable_with<T, U>::value;

template <typename T>
struct is_nothrow_equality_comparable
: is_nothrow_equality_comparable_with<T, T> { };

template <typename T>
constexpr bool is_nothrow_equality_comparable_v =
    is_nothrow_equality_comparable<T>::value;

template <typename T, typename U, typename = void>
struct is_less_than_comparable_with : std::false_type { };

template <typename T, typename U>
struct is_less_than_comparable_with<
    T, U, void_t<decltype(std::declval<T>() < std::declval<U>())>
> : std::true_type { };

template <typename T, typename U>
constexpr bool is_less_than_comparable_with_v =
    is_less_than_comparable_with<T, U>::value;

template <typename T>
struct is_less_than_comparable : is_less_than_comparable_with<T, T> { };

template <typename T>
constexpr bool is_less_than_comparable_v =
    is_less_than_comparable<T>::value;

template <typename T, typename U, bool = is_less_than_comparable_with_v<T, U>>
struct is_nothrow_less_than_comparable_with : std::false_type { };

template <typename T, typename U>
struct is_nothrow_less_than_comparable_with<T, U, true>
: true_type_if_t<noexcept(std::declval<T>() < std::declval<U>())> { };

template <typename T, typename U>
constexpr bool is_nothrow_less_than_comparable_with_v =
    is_nothrow_less_than_comparable_with<T, U>::value;

template <typename T>
struct is_nothrow_less_than_comparable
: is_nothrow_less_than_comparable_with<T, T> { };

template <typename T>
constexpr bool is_nothrow_less_than_comparable_v =
    is_nothrow_less_than_comparable<T>::value;

template <typename T>
struct tuple_size : std::tuple_size<std::remove_reference_t<T>> { };

template <typename T>
constexpr usize tuple_size_v = tuple_size<T>::value;

template <typename...>
struct conjunction : std::true_type { };

template <typename T>
struct conjunction<T> : T { };

template <typename T, typename ...Ts>
struct conjunction<T, Ts...>
: std::conditional_t<static_cast<bool>(T::value), conjunction<Ts...>, T> { };

template <typename ...Ts>
constexpr bool conjunction_v = conjunction<Ts...>::value;

template <typename...>
struct disjunction : std::false_type { };

template <class T>
struct disjunction<T> : T { };

template <class T, class... Ts>
struct disjunction<T, Ts...>
: std::conditional_t<static_cast<bool>(T::value), T, disjunction<Ts...>> { };

template <typename ...Ts>
constexpr bool disjunction_v = disjunction<Ts...>::value;

} // namespace umigv

#endif
