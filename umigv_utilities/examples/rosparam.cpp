#include "umigv_utilities/ros.hpp"
#include "umigv_utilities/types.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>

using namespace umigv::types;

void value_or_eval(ros::NodeHandle &node) {
    umigv::ParameterServer params{ false, node };

    const auto fallback = []{ return 0.0; };

    const auto rate = params["rate"].value_or_eval<f64>(fallback);
}

void value_or_throw() {
    umigv::ParameterServer params{ false, "~" };

    std::string frame_id;

    try {
        frame_id = params["frame_id"].value_or_throw<std::string>();
    } catch (const umigv::ParameterNotFoundException &e) {
        throw;
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "umigv_utilities_rosparam_test");
    ros::NodeHandle node;
    umigv::ParameterServer params{ false, "~" };

    const auto val = params.get<f64>("val").value_or(0.0);
    const std::string name = params.get<std::string>("name").value_or("bob");

    params.enable_caching();

    const auto names = params.get_names().value();
    const auto covariance = params.get<std::vector<f64>>("covariance").value();

    const auto rate = params["rate"].value<f64>();
    const auto frequency = params["frequency"].value_or(100.0);

    value_or_eval(node);
    value_or_throw();
}
