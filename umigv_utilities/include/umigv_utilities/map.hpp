#ifndef UMIGV_MAP_HPP
#define UMIGV_MAP_HPP

// range helper; provided a range or iterator pair and a callable object
// (e.g. a function pointer, lambda, functor), will return the result of calling
// the function on the object

#include "umigv_utilities/types.hpp"
#include "umigv_utilities/invoke.hpp"
#include "umigv_utilities/traits.hpp"
#include "umigv_utilities/utility.hpp"

#include <iterator> // std::begin, std::end, std::iterator_traits
#include <type_traits> // std::common_type_t
#include <utility> // std::forward, std::initializer_list, std::declval

namespace umigv {

template <
    typename I, typename C,
    std::enable_if_t<
        is_input_iterator_v<I>
        && is_invocable_v<C, iterator_reference_t<I>>, int
    > = 0
>
class MappedRange;

template <
    typename I, typename C,
    std::enable_if_t<
        is_input_iterator_v<I>
        && is_invocable_v<C, iterator_reference_t<I>>, int
    > = 0
>
class MappedRangeIterator {
public:
    friend MappedRange<I, C>;

    using difference_type = isize;
    using value_type =
        std::decay_t<invoke_result_t<C, iterator_reference_t<I>>>;
    using pointer = const value_type*;
    using reference = invoke_result_t<C, iterator_reference_t<I>>;
    using iterator_category = std::input_iterator_tag;

    constexpr MappedRangeIterator(const MappedRangeIterator &other)
    noexcept(std::is_nothrow_copy_constructible<I>::value)
    : current_{ other.current_ }, last_{ other.last_ },
      callable_{ std::forward<C>(other.callable_) } { }

    constexpr MappedRangeIterator(MappedRangeIterator &&other)
    noexcept(std::is_nothrow_copy_constructible<I>::value)
    : current_{ std::move(other.current_) }, last_{ std::move(other.last_) },
      callable_{ std::forward<C>(other.callable_) } { }

    constexpr MappedRangeIterator& operator++() {
        if (current_ == last_) {
            throw std::out_of_range{ "MappedRangeIterator::operator++" };
        }

        ++current_;

        return *this;
    }

    constexpr reference operator*() const {
        if (current_ == last_) {
            throw std::out_of_range{ "MappedRangeIterator::operator*" };
        }

        return invoke(std::forward<C>(callable_), *current_);
    }

    constexpr friend bool operator==(const MappedRangeIterator &lhs,
                                     const MappedRangeIterator &rhs) {
        if (lhs.last_ != rhs.last_) {
            throw std::out_of_range{
                "operator==(const MappedRangeIterator&, "
                "const MappedRangeIterator&)"
            };
        }

        return lhs.current_ == rhs.current_;
    }

    constexpr friend bool operator!=(const MappedRangeIterator &lhs,
                                     const MappedRangeIterator &rhs) {
        return !(lhs == rhs);
    }

private:
    constexpr MappedRangeIterator(I current, I last, C &&callable)
    noexcept(std::is_nothrow_move_constructible<I>::value
             && std::is_nothrow_move_constructible<C>::value)
    : current_{ std::move(current) }, last_{ std::move(last) },
      callable_{ std::forward<C>(callable) } { }

    I current_;
    I last_;
    C &&callable_;
};

template <
    typename I, typename C,
    std::enable_if_t<
        is_input_iterator_v<I>
        && is_invocable_v<C, iterator_reference_t<I>>, int
    >
>
class MappedRange {
public:
    using iterator = MappedRangeIterator<I, C>;
    using difference_type = iterator_difference_type_t<iterator>;

    MappedRange(I first, I last, C &&callable)
        : first_{ std::move(first) }, last_{ std::move(last) },
          callable_{ std::forward<C>(callable) }
    { }

    iterator begin() const
    noexcept(std::is_nothrow_constructible<iterator, I, I, C>::value) {
        return iterator{ first_, last_, std::forward<C>(callable_) };
    }

    iterator end() const
    noexcept(std::is_nothrow_constructible<iterator, I, I, C>::value) {
        return iterator{ last_, last_, std::forward<C>(callable_) };
    }

private:
    I first_;
    I last_;
    C &&callable_;
};

template <typename Iterator, typename Function>
typename MappedRange<Iterator, Function>::Iterator
begin(const MappedRange<Iterator, Function> &range)
    noexcept(noexcept(
        std::declval<const MappedRange<Iterator, Function>&>().begin()
    ))
{
    return range.begin();
}

template <typename Iterator, typename Function>
typename MappedRange<Iterator, Function>::Iterator
end(const MappedRange<Iterator, Function> &range)
    noexcept(noexcept(
        std::declval<const MappedRange<Iterator, Function>&>().end()
    ))
{
    return range.end();
}

template <typename R, typename C,
          std::enable_if_t<is_range_v<R>, int> = 0>
auto map(R &&range, C &&callable) {
    using RangeT = MappedRange<begin_result_t<R>, C>;

    return RangeT{ adl_begin(std::forward<R>(range)),
                   adl_end(std::forward<R>(range)),
                   std::forward<C>(callable) };
}

template <typename T, typename C>
auto map(const std::initializer_list<T> list, C &&callable) {
    using RangeT = MappedRange<typename std::initializer_list<T>::iterator, C>;

    return RangeT{ adl_begin(list), adl_end(list),
                   std::forward<C>(callable) };
}

} // namespace umigv

#endif
