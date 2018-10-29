#ifndef UMIGV_EXCEPTIONS_HPP
#define UMIGV_EXCEPTIONS_HPP

// exception types used in UMIGV nodes

#include <phidgets_api/phidget.h> // phidgets::Phidget

#include <stdexcept> // std::runtime_error
#include <string> // std::string
#include <memory> // std::shared_ptr

namespace umigv {

class ParameterNotFoundException : public std::runtime_error {
public:
    ParameterNotFoundException(const std::string &what,
                               const std::string &parameter)
    : std::runtime_error{ what },
      parameter_{ std::make_shared<std::string>(parameter) } { }

    ParameterNotFoundException(const std::string &what,
                               const char *const parameter)
    : std::runtime_error{ what },
      parameter_{ std::make_shared<std::string>(parameter) } { }

    ParameterNotFoundException(const char *const what,
                               const std::string &parameter)
    : std::runtime_error{ what },
      parameter_{ std::make_shared<std::string>(parameter) } { }

    ParameterNotFoundException(const char *const what,
                               const char *const parameter)
    : std::runtime_error{ what },
      parameter_{ std::make_shared<std::string>(parameter) } { }

    virtual ~ParameterNotFoundException() = default;

    const char* what() const noexcept override {
        return std::runtime_error::what();
    }

    virtual const char* parameter() const noexcept {
        return parameter_->c_str();
    }

private:
    std::shared_ptr<std::string> parameter_;
};

class PhidgetsException : public std::runtime_error {
public:
    PhidgetsException(const std::string &what, const int error_code)
    : std::runtime_error{ what }, description_{ make_description(error_code) },
      error_code_{ error_code } { }

    PhidgetsException(const char *const what, const int error_code)
    : std::runtime_error{ what }, description_{ make_description(error_code) },
      error_code_{ error_code } { }

    virtual ~PhidgetsException() = default;

    const char* what() const noexcept override {
        return std::runtime_error::what();
    }

    virtual int error_code() const noexcept {
        return error_code_;
    }

    virtual const char* error_description() const noexcept {
        return description_->c_str();
    }

private:
    static inline std::shared_ptr<std::string>
    make_description(const int code) {
        return std::make_shared<std::string>(
            phidgets::Phidget::getErrorDescription(code)
        );
    }

    std::shared_ptr<std::string> description_;
    int error_code_;
};

} // namespace umigv

#endif
