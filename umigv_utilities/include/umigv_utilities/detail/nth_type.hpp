#ifndef UMIGV_DETAIL_NTH_TYPE_HPP
#define UMIGV_DETAIL_NTH_TYPE_HPP

#include "../types.hpp"

#include <tuple>

namespace umigv {
namespace detail {

template <usize N, typename ...Types>
using NthT = std::tuple_element_t<N, std::tuple<Types...>>;

} // detail
} // namespace umigv

#endif
