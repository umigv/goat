#ifndef PHSPAT_IMPL_ERR_H
#define PHSPAT_IMPL_ERR_H

#include <cstddef>
#include <string>
#include <system_error>

namespace ph {

class PhidgetCategory : public std::error_category {
public:
    PhidgetCategory(const PhidgetCategory &other) = delete;

    PhidgetCategory(PhidgetCategory &&other) = delete;

    PhidgetCategory& operator=(const PhidgetCategory &other) = delete;

    PhidgetCategory& operator=(PhidgetCategory &&other) = delete;

    static const PhidgetCategory& instance() noexcept;

    const char* name() const noexcept override;

    std::string message(int condition) const override;

private:
    constexpr PhidgetCategory() noexcept = default;

    virtual ~PhidgetCategory() = default;
};

class PhidgetErrorEventCategory : public std::error_category {
public:
    constexpr explicit PhidgetErrorEventCategory(const char *description) noexcept
    : description_{ description } { }

    virtual ~PhidgetErrorEventCategory() = default;

    const char* name() const noexcept override;

    std::string message(int condition) const override;

    int compare(const PhidgetErrorEventCategory &other) const noexcept;

    int compare(const char *description) const noexcept;

private:
    const char* description_;
};

} // namespace ph

#endif
