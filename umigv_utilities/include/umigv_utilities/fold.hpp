#ifndef UMIGV_FOLD_HPP
#define UMIGV_FOLD_HPP

#include "umigv_utilities/invoke.hpp"

#include <tuple>
#include <utility>

namespace umigv {

template <typename F, typename T>
constexpr decltype(auto) binary_fold(F&&, T &&t) noexcept {
    return std::forward<T>(t);
}

template <typename F, typename T, typename U, typename ...As,
          std::enable_if_t<is_invocable_v<F, T, U>, int> = 0>
constexpr decltype(auto) binary_fold(F &&f, T &&t, U &&u, As &&...args)
noexcept(is_nothrow_invocable_v<F, T, U>) {
    return binary_fold(std::forward<F>(f),
                       invoke(std::forward<F>(f), std::forward<T>(t),
                              std::forward<U>(u)),
                       std::forward<As>(args)...);
}

template <typename F>
constexpr void unary_fold(F&&) noexcept { }

template <typename F, typename T, typename ...As,
          std::enable_if_t<is_invocable_v<F, T>, int> = 0>
constexpr void unary_fold(F &&f, T &&t, As &&...args)
noexcept(is_nothrow_invocable_v<F, T>) {
    invoke(std::forward<F>(f), std::forward<T>(t));
    unary_fold(std::forward<F>(f), std::forward<As>(args)...);
}

constexpr bool boolean_and() noexcept {
    return true;
}

template <typename ...As>
constexpr bool boolean_and(const bool condition, As &&...args) noexcept {
    if (!condition) {
        return false;
    }

    return boolean_and(std::forward<As>(args)...);
}

constexpr bool boolean_or() noexcept {
    return false;
}

template <typename ...As>
constexpr bool boolean_or(const bool condition, As &&...args) noexcept {
    if (condition) {
        return true;
    }

    return boolean_or(std::forward<As>(args)...);
}

namespace detail {

template <typename F, typename G, typename T, typename U, usize I, usize S>
struct BinaryPairFolder {
public:
    template <typename V>
    constexpr decltype(auto) operator()(V &&v) noexcept {
        auto &&step_return =
            invoke(std::forward<G>(g),
                   std::get<I>(std::forward<T>(tup1)),
                   std::get<I>(std::forward<U>(tup2)));

        auto &&step_fold =
            invoke(std::forward<F>(f), std::forward<V>(v),
                   std::forward<decltype(step_return)>(step_return));

        BinaryPairFolder<F, G, T, U, I + 1, S> folder{ std::forward<F>(f),
                                                       std::forward<G>(g),
                                                       std::forward<T>(tup1),
                                                       std::forward<U>(tup2) };

        return folder(std::forward<decltype(step_fold)>(step_fold));
    }

    F &&f;
    G &&g;
    T &&tup1;
    U &&tup2;
};

template <typename F, typename G, typename T, typename U, usize S>
struct BinaryPairFolder<F, G, T, U, S, S> {
public:
    template <typename V>
    constexpr decltype(auto) operator()(V &&v) noexcept {
        return std::forward<V>(v);
    }

    F &&f;
    G &&g;
    T &&tup1;
    U &&tup2;
};

} // namespace detail

template <
    typename F, typename G, typename T, typename U,
    std::enable_if_t<
        (tuple_size_v<T> == tuple_size_v<U>)
        && (tuple_size_v<T> > 0), int
    > = 0
>
constexpr decltype(auto) binary_fold_pairs(F &&f, G &&g, T &&tup1, U &&tup2) {
    using FolderT = detail::BinaryPairFolder<F, G, T, U, 1, tuple_size_v<T>>;

    FolderT folder{ std::forward<F>(f), std::forward<G>(g),
                    std::forward<T>(tup1), std::forward<U>(tup2) };

    auto &&first_fold =
        invoke(std::forward<G>(g),
               std::get<0>(std::forward<T>(tup1)),
               std::get<0>(std::forward<U>(tup2)));

    return folder(first_fold);
}

} // namespace umigv

#endif
