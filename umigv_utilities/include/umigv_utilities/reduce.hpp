#ifndef UMIGV_REDUCE_HPP
#define UMIGV_REDUCE_HPP

#include "umigv_utilities/invoke.hpp" // detail::invoke

#include <iterator> // std::begin, std::end, std::iterator_traits
#include <utility> // std::forward

namespace umigv {

template <typename Begin, typename End, typename Operator,
          typename = std::common_type_t<Begin, End>>
decltype(auto) reduce(Begin begin, End end, Operator &&op) {
    using IteratorT = std::common_type_t<Begin, End>;
    using ValueT = typename std::iterator_traits<IteratorT>::value_type;

    if (begin == end) {
        return ValueT{ };
    }

    ValueT accumulator = *begin;

    ++begin;

    while (begin != end) {
        accumulator = invoke(std::forward<Operator>(op), accumulator, *begin);

        ++begin;
    }

    return accumulator;
}

template <typename Range, typename Operator>
decltype(auto) reduce(Range &&range, Operator &&op) {
    using std::begin;
    using std::end;

    return reduce(begin(std::forward<Range>(range)),
                  end(std::forward<Range>(range)),
                  std::forward<Operator>(op));
}

template <typename T, typename Operator>
decltype(auto) reduce(const std::initializer_list<T> list, Operator &&op) {
    using std::begin;
    using std::end;

    return reduce(begin(list), end(list), std::forward<Operator>(op));
}

} // namespace umigv

#endif
