#ifndef UMIGV_ITERATOR_RANGE_HPP
#define UMIGV_ITERATOR_RANGE_HPP

#include "umigv_utilities/traits.hpp"

namespace umigv {

template <typename I, std::enable_if_t<is_iterator_v<I>, int> = 0>
class IteratorRange {
public:
    using iterator = I;
    using difference_type = iterator_difference_type_t<I>;

    constexpr IteratorRange(I first, I last)
    noexcept(std::is_nothrow_move_constructible<I>::value)
    : first_{ std::move(first) }, last_{ std::move(last) } { }

    constexpr iterator begin() const
    noexcept(std::is_nothrow_copy_constructible<I>::value) {
        return first_;
    }

    constexpr iterator end() const
    noexcept(std::is_nothrow_copy_constructible<I>::value) {
        return last_;
    }

private:
    I first_;
    I last_;
};

template <typename I>
constexpr IteratorRange<I> iterator_range(I first, I last)
noexcept(std::is_nothrow_constructible<IteratorRange<I>, I, I>::value) {
    return { std::move(first), std::move(last) };
}

} // namespace umigv

#endif
