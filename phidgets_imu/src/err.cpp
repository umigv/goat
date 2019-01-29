#include "err.h"

#include <cstring>
#include <functional>
#include <mutex>
#include <set>
#include <type_traits>

#include <phidget22.h>

namespace ph {

const PhidgetCategory& PhidgetCategory::instance() noexcept {
    static const PhidgetCategory category;

    return category;
}

const char* PhidgetCategory::name() const noexcept {
    return "ph::PhidgetCategory";
}

std::string PhidgetCategory::message(int condition) const {
    const char *msg;
    const auto ret{
        Phidget_getErrorDescription(static_cast<PhidgetReturnCode>(condition), &msg)
    };

    if (ret != EPHIDGET_OK) {
        std::abort();
    }

    return { msg };
}

const char* PhidgetErrorEventCategory::name() const noexcept {
    return "ph::PhidgetErrorEventCategory";
}

constexpr const char* get_error_event_code_description(Phidget_ErrorEventCode code) {
    switch (code) {
        case EEPHIDGET_BADVERSION: return "Version Mismatch";
        case EEPHIDGET_BUSY: return "Phidget in Use";
        case EEPHIDGET_NETWORK: return "Network Error";
        case EEPHIDGET_DISPATCH: return "Dispatch Error";
        case EEPHIDGET_FAILURE: return "General Failure";
        case EEPHIDGET_OK: return "Error Cleared";
        case EEPHIDGET_OVERRUN: return "Sample Overrun";
        case EEPHIDGET_PACKETLOST: return "Packet Lost";
        case EEPHIDGET_WRAP: return "Wrap-Around";
        case EEPHIDGET_OVERTEMP: return "Over-Temperature";
        case EEPHIDGET_OVERCURRENT: return "Over-Current";
        case EEPHIDGET_OUTOFRANGE: return "Out of Range";
        case EEPHIDGET_BADPOWER: return "Bad Power";
        case EEPHIDGET_SATURATION: return "Saturation";
        case EEPHIDGET_OVERVOLTAGE: return "Over-Voltage";
        case EEPHIDGET_FAILSAFE: return "Fail-Safe";
        case EEPHIDGET_VOLTAGEERROR: return "Voltage Error";
        case EEPHIDGET_ENERGYDUMP: return "Energy Dump";
        case EEPHIDGET_MOTORSTALL: return "Motor Stall";
    }
}

std::string PhidgetErrorEventCategory::message(int condition) const {
    const auto code{ static_cast<Phidget_ErrorEventCode>(condition) };

    std::string msg{ get_error_event_code_description(code) };
    msg += ": ";
    msg += description_;

    return msg;
}

int PhidgetErrorEventCategory::compare(const PhidgetErrorEventCategory &other) const noexcept {
    return std::strcmp(description_, other.description_);
}

int PhidgetErrorEventCategory::compare(const char *description) const noexcept {
    return std::strcmp(description_, description);
}

} // namespace ph
