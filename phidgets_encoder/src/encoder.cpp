#include "encoder.h"

#include <algorithm>
#include <cassert>
#include <sstream>

/** Throw if a Phidget function doesn't return EPHIDGET_OK. */
#define EXPECT(...) do_expect(__VA_ARGS__, __FILE__, __func__, __LINE__)

namespace {

void do_expect(PhidgetReturnCode ret, const char *file, const char *func,
               int line) {
  if (ret == EPHIDGET_OK) {
    return;
  }

  const char *what;
  Phidget_getErrorDescription(ret, &what);

  std::ostringstream oss;
  oss << file << ':' << func << ':' << line << ": " << what << " ("
      << static_cast<int>(ret) << ')';

  throw std::runtime_error(oss.str());
}

} // namespace

/**
 *  Blocks until a Phidget can be attached to.
 *
 *  @param channel_index The index of the channel that this Encoder will
 *                       attach to.
 *  @returns an Encoder that is attached to the specified channel.
 */
Encoder::Encoder(int channel_index) {
  EXPECT(PhidgetEncoder_create(&channel_));

  EXPECT(Phidget_setOnAttachHandler(as_phidget(), Encoder::on_attach, this));
  EXPECT(Phidget_setOnDetachHandler(as_phidget(), Encoder::on_detach, this));
  EXPECT(Phidget_setOnErrorHandler(as_phidget(), Encoder::on_error, this));
  EXPECT(Phidget_setChannel(as_phidget(), channel_index));

  EXPECT(Phidget_openWaitForAttachment(as_phidget(), 0));

  std::uint32_t data_interval;
  EXPECT(PhidgetEncoder_getMinDataInterval(channel_, &data_interval));
  EXPECT(PhidgetEncoder_setDataInterval(channel_, data_interval));
  EXPECT(PhidgetEncoder_setOnPositionChangeHandler(
      channel_, Encoder::on_position_change, this));
}

/** Close and clean up the associated Phidget handle. */
Encoder::~Encoder() {
  // if this throws, the program aborts
  // can't think of any other reasonable action to take in this scenario
  EXPECT(PhidgetEncoder_delete(&channel_));
}

/**
 *  Positive position refers to counterclockwise rotation and negative
 *  position refers to clockwise rotation. Position is maintained even in the
 *  event of the associated Phidget detaching.
 *
 *  @returns the number of ticks this encoder has rotated since startup.
 */
std::int64_t Encoder::position() const noexcept { return position_.load(); }

/** @returns the serial number of the board this Encoder is attached to. */
std::int32_t Encoder::serial_number() const {
  std::int32_t sn;

  EXPECT(Phidget_getDeviceSerialNumber(as_phidget(), &sn));

  return sn;
}

/** @returns true if this Encoder is attached to a device. */
bool Encoder::is_attached() const noexcept { return is_attached_.load(); }

/**
 *  @returns a list of the errors that have occured since the last call to
 *  this function.
 */
std::vector<std::pair<Phidget_ErrorEventCode, const char *>>
Encoder::errors() noexcept {
  std::vector<std::pair<Phidget_ErrorEventCode, const char *>> errors;

  {
    const std::lock_guard<std::mutex> guard(errors_mtx_);
    errors.swap(errors_);
  }

  return errors;
}

PhidgetHandle Encoder::as_phidget() const noexcept {
  return reinterpret_cast<PhidgetHandle>(channel_);
}

void Encoder::on_position_change(PhidgetEncoderHandle, void *self,
                                 int position_change, double, int) noexcept {
  // phidgets uses + rotation to mean cw rotation
  // we want + rotation to mean ccw rotation
  reinterpret_cast<Encoder *>(self)->position_.fetch_sub(position_change);
}

void Encoder::on_attach(PhidgetHandle, void *self) noexcept {
  reinterpret_cast<Encoder *>(self)->is_attached_.store(true);
}

void Encoder::on_detach(PhidgetHandle, void *self) noexcept {
  reinterpret_cast<Encoder *>(self)->is_attached_.store(false);
}

void Encoder::on_error(PhidgetHandle, void *self, Phidget_ErrorEventCode errc,
                       const char *what) {
  Encoder &e = *reinterpret_cast<Encoder *>(self);

  const std::lock_guard<std::mutex> guard(e.errors_mtx_);
  e.errors_.emplace_back(errc, what);
}
