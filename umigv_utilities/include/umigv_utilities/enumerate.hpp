#ifndef UMIGV_ENUMERATE_HPP
#define UMIGV_ENUMERATE_HPP

// range helper; provided a range or iterator pair, will enumerate each
// element upon dereferencing

#include "umigv_utilities/types.hpp" // umigv::usize, umigv::isize

#include <iterator> // std::begin, std::end, std::iterator_traits
#include <type_traits> // std::common_type_t
#include <utility> // std::forward, std::initializer_list, std::pair

namespace umigv {

// lazy evaluation iterator that return a pair
template <typename It, typename Count = usize>
class EnumeratedRange {
public:
    class Iterator {
    public:
        friend EnumeratedRange;

        using difference_type = isize;
        using value_type = std::pair<Count, decltype((*std::declval<It>()))>;
        using pointer = const value_type*;
        using reference = const value_type&;
        using iterator_category = std::forward_iterator_tag;

        Iterator& operator++() noexcept(noexcept(++std::declval<It&>())
                                        and noexcept(++std::declval<Count&>())) {
            ++current_;
            ++index_;

            return *this;
        }

        value_type operator*() const
            noexcept(noexcept(value_type{ std::declval<Count>(),
                                          *std::declval<It>() }))
        {
            return value_type{ index_, *current_ };
        }

        friend bool operator==(const Iterator lhs, const Iterator rhs)
            noexcept(noexcept(std::declval<It>() == std::declval<It>()))
        {
            return lhs.current_ == rhs.current_;
        }

        friend bool operator!=(const Iterator lhs, const Iterator rhs)
            noexcept(noexcept(std::declval<It>() != std::declval<It>()))
        {
            return lhs.current_ != rhs.current_;
        }

    private:
        explicit Iterator(It current)
            noexcept(noexcept(It{ std::move(std::declval<It>()) })
                     and noexcept(Count{ }))
            : current_{ std::move(current) }, index_{ }
        { }

        It current_;
        Count index_;
    };

    EnumeratedRange(It begin, It end)
        noexcept(noexcept(It{ std::move(std::declval<It>()) }))
        : begin_{ std::move(begin) }, end_{ std::move(end) }
    { }

    Iterator begin() const noexcept(noexcept(Iterator{ std::declval<It>() })) {
        return Iterator{ begin_ };
    }

    Iterator end() const noexcept(noexcept(Iterator{ std::declval<It>() })) {
        return Iterator{ end_ };
    }

private:
    It begin_;
    It end_;
};

template <typename Iterator, typename Count>
typename EnumeratedRange<Iterator, Count>::Iterator
begin(const EnumeratedRange<Iterator, Count> &range) noexcept {
    return range.begin();
}

template <typename Iterator, typename Count>
typename EnumeratedRange<Iterator, Count>::Iterator
end(const EnumeratedRange<Iterator, Count> &range) noexcept {
    return range.end();
}

template <typename Count = usize, typename Range>
auto enumerate(Range &&range) {
    using std::begin;
    using std::end;

    using IteratorT = decltype(begin(std::forward<Range>(range)));
    using RangeT = EnumeratedRange<IteratorT, Count>;

    return RangeT{ begin(std::forward<Range>(range)),
                   end(std::forward<Range>(range)) };
}

template <typename Count = usize, typename Begin, typename End,
          typename = std::common_type_t<Begin, End>>
auto enumerate(Begin begin, End end) {
    using IteratorT = std::common_type_t<Begin, End>;
    using RangeT = EnumeratedRange<IteratorT, Count>;

    return RangeT{ begin, end };
}

template <typename Count = usize, typename T>
auto enumerate(const std::initializer_list<T> list) {
    using std::begin;
    using std::end;

    using IteratorT = decltype(begin(list));
    using RangeT = EnumeratedRange<IteratorT, Count>;

    return RangeT{ begin(list), end(list) };
}

} // namespace umigv

#endif
