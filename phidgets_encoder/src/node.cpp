#include "encoder.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

struct EncoderDescription {
  int cpr;
  int channel;
  std::string name;
  std::unique_ptr<Encoder> encoder_ptr;

  /** @returns true if this EncoderDescription is attached to a Phidget. */
  bool is_attached() const noexcept {
    return encoder_ptr && encoder_ptr->is_attached();
  }
};

/**
 *  Get the rate (in Hz) to publish at.
 *
 *  Defaults to 125 if not present or nonpositive.
 *
 *  @param nh Should be this node's private NodeHandle.
 */
static double get_rate(ros::NodeHandle &nh);

/**
 *  Build the set of EncoderDescriptions from parameters.
 *
 *  Defaults to publishing channels 0 and 1 at 1440 CPR as joints "wheel0" and
 *  "wheel1". Channels to skip can be specified using -1. For example:
 *
 *  counts_per_revolution: [-1, 360]
 *  names: ["", "wheel0"]
 *
 *  This will result in one EncoderDescription, "wheel0", that listens to
 *  channel 0 and rotates at 360 CPR.
 */
static std::vector<EncoderDescription> get_descriptions(ros::NodeHandle &nh);

int main(int argc, char **argv) {
  ros::init(argc, argv, "phidgets_encoder_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv("~");

  const ros::Rate rate(get_rate(priv));
  std::vector<EncoderDescription> desc;

  // a little kitsch, but it works. messy to have a small try-catch like this
  try {
    desc = get_descriptions(priv);
  } catch (const std::exception &e) {
    ROS_FATAL("%s", e.what());

    return EXIT_FAILURE;
  }

  if (desc.empty()) {
    ROS_INFO("no encoders provided, exiting...");

    return EXIT_SUCCESS;
  }

  for (EncoderDescription &e : desc) {
    e.encoder_ptr = std::make_unique<Encoder>(e.channel);

    const int serial_number = e.encoder_ptr->serial_number();
    ROS_INFO("attached to Phidget #%d, channel %d", serial_number, e.channel);
  }

  ROS_INFO("connected to Phidget #%d",
           static_cast<int>(desc.front().encoder_ptr->serial_number()));

  auto pub = nh.advertise<sensor_msgs::JointState>("encoders/state", 10);

  auto timer_callback = [&desc, &pub](const ros::TimerEvent &) {
    // don't publish if encoders are detached; this is a failure condition

    for (const EncoderDescription &e : desc) {
      if (!e.is_attached()) {
        ROS_WARN_THROTTLE(1,
                          "channel %d associated with joint %s is not attached",
                          e.channel, e.name.c_str());

        return;
      }
    }

    sensor_msgs::JointState msg;
    static std::uint32_t seq = 0;
    msg.header.seq = seq++;
    msg.header.stamp = ros::Time::now();

    msg.name.reserve(desc.size());
    msg.position.reserve(desc.size());

    for (EncoderDescription &e : desc) {
      assert(e.encoder_ptr);

      for (const auto &err : e.encoder_ptr->errors()) {
        ROS_WARN("Phidget error: %s", err.second);
      }

      const std::int64_t pos = e.encoder_ptr->position();

      // convert tick count to radians
      constexpr double PI = 3.14159265358979323;
      const double rotation = static_cast<double>(pos) / e.cpr * 2 * PI;

      msg.name.push_back(e.name);
      msg.position.push_back(rotation);
    }

    // MIGHT be faster to use a shared_ptr and nodelet logic here
    // but does it matter?
    pub.publish(msg);
  };

  auto timer = nh.createTimer(rate, &decltype(timer_callback)::operator(),
                              &timer_callback);

  ros::spin();
}

/**
 *  Get the rate (in Hz) to publish at.
 *
 *  Defaults to 125 if not present or nonpositive.
 *
 *  @param nh Should be this node's private NodeHandle.
 */
static double get_rate(ros::NodeHandle &nh) {
  constexpr double DEFAULT_RATE = 125;

  double rate = nh.param("rate", DEFAULT_RATE);

  if (rate <= 0) {
    ROS_WARN("requested rate (%f Hz) out of range, defaulting to %f Hz", rate,
             DEFAULT_RATE);

    return DEFAULT_RATE;
  } else {
    ROS_INFO("using rate of %f Hz", rate);
  }

  return rate;
}

/**
 *  Build the set of EncoderDescriptions from parameters.
 *
 *  Defaults to publishing channels 0 and 1 at 1440 CPR as joints "wheel0" and
 *  "wheel1". Channels to skip can be specified using -1. For example:
 *
 *  counts_per_revolution: [-1, 360]
 *  names: ["", "wheel0"]
 *
 *  This will result in one EncoderDescription, "wheel0", that listens to
 *  channel 0 and rotates at 360 CPR.
 */
static std::vector<EncoderDescription> get_descriptions(ros::NodeHandle &nh) {
  // sparse array, kind of
  std::vector<int> counts_per_revolution =
      nh.param("counts_per_revolution", std::vector<int>{1440, 1440});

  constexpr int SKIP = -1;

  for (int &cpr : counts_per_revolution) {
    if (cpr != SKIP && cpr < 0) {
      ROS_WARN("requested CPR (%d) out of range, using 1440", cpr);
      cpr = 1440;
    }
  }

  // filter-map from cpr to descriptions
  std::vector<EncoderDescription> desc;
  for (std::size_t i = 0; i < counts_per_revolution.size(); ++i) {
    if (counts_per_revolution[i] != SKIP) {
      desc.push_back({counts_per_revolution[i], static_cast<int>(i)});
    }
  }

  std::vector<std::string> names =
      nh.param("names", std::vector<std::string>{"wheel0", "wheel1"});

  if (names.size() != desc.size()) {
    throw std::runtime_error(
        "~/cpr must have the same number of valid elements as ~/names");
  }

  for (std::size_t i = 0; i < desc.size(); ++i) {
    desc[i].name = std::move(names[i]);

    ROS_INFO("publishing channel %d as %s with %d CPR", static_cast<int>(i),
             desc[i].name.c_str(), desc[i].cpr);
  }

  return desc;
}
