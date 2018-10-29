#ifndef UMIGV_PAIRS_HPP
#define UMIGV_PAIRS_HPP

// defines umigv::pairs, which returns pairs of elements in a range

#include "umigv_utilities/types.hpp"
#include "umigv_utilities/utility.hpp"

#include <iterator>
#include <memory>
#include <type_traits>
#include <utility>

#include <boost/optional.hpp>

namespace umigv {

template <typename Iterator>
class PairedRange;

template <typename Iterator>
class PairedRangeIterator {
    using IterRefT = typename std::iterator_traits<Iterator>::reference;
public:
    friend PairedRange<Iterator>;

    using difference_type = isize;
    using value_type = std::pair<IterRefT, IterRefT>;
    using pointer = const value_type*;
    using reference = const value_type&;
    using iterator_category = std::forward_iterator_tag;

    constexpr PairedRangeIterator()
    noexcept(std::is_nothrow_default_constructible<Iterator>::value) = default;

    constexpr PairedRangeIterator& operator++() {
        if (trailer_ == last_ || leader_ == last_) {
            throw std::out_of_range{ "PairedRangeIterator::operator*" };
        }

        ++trailer_;
        ++leader_;

        return *this;
    }

    constexpr PairedRangeIterator operator++(int) {
        PairedRangeIterator to_return = *this;

        ++(*this);

        return to_return;
    }

    constexpr reference operator*() const {
        if (trailer_ == last_ || leader_ == last_) {
            throw std::out_of_range{ "PairedRangeIterator::operator*" };
        }

        deref_.emplace(*trailer_, *leader_);

        return deref_.value();
    }

    constexpr pointer operator->() const {
        return std::addressof(**this);
    }

    friend constexpr bool operator==(const PairedRangeIterator lhs,
                                     const PairedRangeIterator rhs) {
        if (lhs.last_ != rhs.last_) {
            throw std::out_of_range{
                "operator==(PairedRangeIterator, PairedRangeIterator)"
            };
        }

        return lhs.trailer_ == rhs.trailer_ || lhs.leader_ == rhs.leader_;
    }

    friend constexpr bool operator!=(const PairedRangeIterator lhs,
                                     const PairedRangeIterator rhs) {
        return !(lhs == rhs);
    }

private:
    constexpr PairedRangeIterator(const Iterator current, const Iterator last)
    : last_{ last }, deref_{ boost::none } {
        if (current != last && std::next(current) != last) {
            trailer_ = current;
            leader_ = std::next(current);
        } else {
            trailer_ = last;
            leader_ = last;
        }
    }

    Iterator trailer_;
    Iterator leader_;
    Iterator last_;
    mutable boost::optional<value_type> deref_ = boost::none;
};

template <typename Iterator>
class PairedRange {
public:
    using iterator = PairedRangeIterator<Iterator>;
    using difference_type =
        typename std::iterator_traits<iterator>::difference_type;

    constexpr PairedRange(Iterator first, Iterator last)
    noexcept(std::is_nothrow_copy_constructible<Iterator>::value)
    : first_{ first }, last_{ last } { }

    constexpr iterator begin() const {
        return { first_, last_ };
    }

    constexpr iterator end() const {
        return { last_, last_ };
    }

private:
    Iterator first_;
    Iterator last_;
};

template <typename Iterator>
constexpr PairedRangeIterator<Iterator>
begin(const PairedRange<Iterator> &range) {
    return range.begin();
}

template <typename Iterator>
constexpr PairedRangeIterator<Iterator>
end(const PairedRange<Iterator> &range) {
    return range.end();
}

template <typename Range>
PairedRange<decltype(adl_begin(std::declval<Range>()))> pairs(Range &&range) {
    return { adl_begin(std::forward<Range>(range)),
             adl_end(std::forward<Range>(range)) };
}

} // namespace umigv

#endif
