#ifndef ENCODER_H
#define ENCODER_H

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

#include <phidget22.h>

/**
 *  Encoder wraps a PhidgetEncoderHandle to allow convenient concurrent use.
 *
 *  Encoder is immobile (noncopyable and immovable), as it registers callbacks
 *  based on its own address. If this is not desirable, refer to Encoder through
 *  a std::unique_ptr<Encoder>.
 *
 *  Encoder is thread safe. All operations can be executed concurrently.
 */
class Encoder {
public:
  /**
   *  Blocks until a Phidget can be attached to.
   *
   *  @param channel_index The index of the channel that this Encoder will
   *                       attach to.
   *  @returns an Encoder that is attached to the specified channel.
   */
  explicit Encoder(int channel_index);

  Encoder(const Encoder &other) = delete;
  Encoder(Encoder &&other) = delete;

  /** Close and clean up the associated Phidget handle. */
  ~Encoder();

  Encoder &operator=(const Encoder &other) = delete;
  Encoder &operator=(Encoder &&other) = delete;

  /**
   *  Positive position refers to counterclockwise rotation and negative
   *  position refers to clockwise rotation. Position is maintained even in the
   *  event of the associated Phidget detaching.
   *
   *  @returns the number of ticks this encoder has rotated since startup.
   */
  std::int64_t position() const noexcept;

  /** @returns the serial number of the board this Encoder is attached to. */
  std::int32_t serial_number() const;

  /** @returns true if this Encoder is attached to a device. */
  bool is_attached() const noexcept;

  /**
   *  @returns a list of the errors that have occured since the last call to
   *  this function.
   */
  std::vector<std::pair<Phidget_ErrorEventCode, const char *>>
  errors() noexcept;

private:
  PhidgetHandle as_phidget() const noexcept;

  static void on_position_change(PhidgetEncoderHandle, void *self,
                                 int position_change, double, int) noexcept;

  static void on_attach(PhidgetHandle, void *self) noexcept;

  static void on_detach(PhidgetHandle, void *self) noexcept;

  static void on_error(PhidgetHandle, void *self, Phidget_ErrorEventCode errc,
                       const char *what);

  PhidgetEncoderHandle channel_;
  std::atomic<std::int64_t> position_ = ATOMIC_VAR_INIT(0);
  std::atomic<bool> is_attached_ = ATOMIC_VAR_INIT(false);
  std::mutex errors_mtx_;
  std::vector<std::pair<Phidget_ErrorEventCode, const char *>> errors_;
};

#endif
