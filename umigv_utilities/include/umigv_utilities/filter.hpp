#ifndef UMIGV_FILTER_HPP
#define UMIGV_FILTER_HPP

// dereferencing filter returns only elements that evaluate true by a predicate

#include "umigv_utilities/invoke.hpp"
#include "umigv_utilities/traits.hpp"
#include "umigv_utilities/types.hpp"
#include "umigv_utilities/utility.hpp"

#include <iterator>
#include <type_traits>
#include <utility>

namespace umigv {

template <
    typename I, typename P,
    std::enable_if_t<is_iterator_v<I>
                     && is_invocable_v<P, iterator_reference_t<I>>, int> = 0
>
class FilteredRange;

template <
    typename I, typename P,
    std::enable_if_t<is_iterator_v<I>
                     && is_invocable_v<P, iterator_reference_t<I>>, int> = 0
>
class FilteredRangeIterator {
public:
    friend FilteredRange<I, P>;

    using difference_type = iterator_difference_type_t<I>;
    using value_type = iterator_value_type_t<I>;
    using pointer = iterator_pointer_t<I>;
    using reference = iterator_reference_t<I>;
    using iterator_category = std::input_iterator_tag;

    constexpr FilteredRangeIterator(const FilteredRangeIterator &other)
    noexcept(std::is_nothrow_copy_constructible<I>::value)
    : current_{ other.current_ }, last_{ other.last_ },
      predicate_{ std::forward<P>(other.predicate_) } { }

    constexpr FilteredRangeIterator(FilteredRangeIterator &&other)
    noexcept(std::is_nothrow_move_constructible<I>::value)
    : current_{ std::move(other.current_) },
      last_{ std::move(other.last_) },
      predicate_{ std::forward<P>(other.predicate_) } { }

    constexpr FilteredRangeIterator& operator++()
    noexcept(noexcept(++std::declval<I&>())
             && noexcept(std::declval<FilteredRangeIterator&>().validate())) {
        ++current_;
        validate();

        return *this;
    }

    constexpr FilteredRangeIterator operator++(int)
    noexcept(std::is_nothrow_copy_constructible<I>::value
             && std::is_nothrow_copy_constructible<P>::value
             && noexcept(++std::declval<FilteredRangeIterator&>())) {
        const FilteredRangeIterator to_return = *this;

        ++(*this);

        return to_return;
    }

    constexpr reference operator*() const
    noexcept(noexcept(*std::declval<const I&>())) {
        return *current_;
    }

    constexpr friend bool operator==(const FilteredRangeIterator &lhs,
                                     const FilteredRangeIterator &rhs) {
        if (lhs.last_ != rhs.last_) {
            throw std::out_of_range{
                "operator==(FilteredRangeIterator, FilteredRangeIterator)"
            };
        }

        return lhs.current_ == rhs.current_;
    }

    constexpr friend bool operator!=(const FilteredRangeIterator &lhs,
                                     const FilteredRangeIterator &rhs) {
        return !(lhs == rhs);
    }

private:
    constexpr FilteredRangeIterator(I current, I last, P &&predicate)
    noexcept(std::is_nothrow_move_constructible<I>::value
             && noexcept(std::declval<FilteredRangeIterator&>().validate()))
    : current_{ std::move(current) }, last_{ std::move(last) },
      predicate_{ std::forward<P>(predicate) } {
        validate();
    }

    constexpr void validate()
    noexcept(is_nothrow_equality_comparable_v<I>
             && is_nothrow_invocable_v<P, iterator_reference_t<I>>
             && noexcept(++std::declval<I&>())) {
        for (; !(current_ == last_) && !invoke(std::forward<P>(predicate_),
                                               *current_);
             ++current_) { }
    }

    I current_;
    I last_;
    P &&predicate_;
};

template <
    typename I, typename P,
    std::enable_if_t<is_iterator_v<I>
                     && is_invocable_v<P, iterator_reference_t<I>>, int>
>
class FilteredRange {
public:
    using iterator = FilteredRangeIterator<I, P>;
    using difference_type = iterator_difference_type_t<iterator>;

    constexpr FilteredRange(I first, I last, P &&predicate)
    noexcept(std::is_nothrow_move_constructible<I>::value)
    : first_{ std::move(first) }, last_{ std::move(last) },
      predicate_{ std::forward<P>(predicate) }
    { }

    constexpr iterator begin() const
    noexcept(std::is_nothrow_constructible<iterator, I, I, P>::value
             && std::is_nothrow_copy_constructible<I>::value) {
        return { first_, last_, std::forward<P>(predicate_) };
    }

    constexpr iterator end() const
    noexcept(std::is_nothrow_constructible<iterator, I, I, P>::value
             && std::is_nothrow_copy_constructible<I>::value) {
        return { last_, last_, std::forward<P>(predicate_) };
    }

private:
    I first_;
    I last_;
    P &&predicate_;
};

template <typename I, typename P>
constexpr FilteredRangeIterator<I, P> begin(const FilteredRange<I, P> &range)
noexcept (has_nothrow_begin_v<FilteredRangeIterator<I, P>>) {
    return range.begin();
}

template <typename I, typename P>
constexpr FilteredRangeIterator<I, P> end(const FilteredRange<I, P> &range)
noexcept (has_nothrow_end_v<FilteredRangeIterator<I, P>>) {
    return range.end();
}

template <
    typename R, typename P,
    std::enable_if_t<is_range_v<R>
                     && is_invocable_v<P, range_reference_t<R>>, int> = 0>
constexpr FilteredRange<begin_result_t<R>, P> filter(R &&range, P &&predicate)
noexcept(std::is_nothrow_constructible<
    FilteredRange<begin_result_t<R>, P>,
    begin_result_t<R>,
    end_result_t<R>,
    P
>::value) {
    using RangeT = FilteredRange<begin_result_t<R>, P>;

    return RangeT{ adl_begin(std::forward<R>(range)),
                   adl_end(std::forward<R>(range)),
                   std::forward<P>(predicate) };
}

template <typename T, typename P,
          std::enable_if_t<is_invocable_v<P, const T&>, int> = 0>
constexpr FilteredRange<begin_result_t<std::initializer_list<T>>, P>
filter(const std::initializer_list<T> list, P &&predicate)
noexcept(std::is_nothrow_constructible<
    FilteredRange<begin_result_t<std::initializer_list<T>>, P>,
    begin_result_t<std::initializer_list<T>>,
    end_result_t<std::initializer_list<T>>,
    P
>::value) {
    using RangeT = FilteredRange<begin_result_t<std::initializer_list<T>>, P>;

    return RangeT{ adl_begin(list), adl_end(list),
                   std::forward<P>(predicate) };
}

} // namespace umigv

#endif
