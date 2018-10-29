#include "cv.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <utility>

namespace umigv {

CannyFilter::CannyFilter(const cv::Scalar &lower_bound,
                         const cv::Scalar &upper_bound,
                         const double lower_threshold,
                         const double upper_threshold)
    : lowerBound{lower_bound},
      upperBound{upper_bound},
      lowerThreshold{lower_threshold},
      upperThreshold{upper_threshold} {}

cv::Mat CannyFilter::filter(const cv::Mat &to_filter) {
  // Blur the image to reduce noise
  cv::blur(to_filter, buffer1, cv::Size(3, 3));

  // Convert to HSV
  cv::cvtColor(buffer1, buffer2, cv::COLOR_RGB2HSV);

  // Perform thresholding based on HSV values to isolate "white-ish"
  // pixels White pixels that aren't that bright White pixels that are
  // very bright and have a slight tint
  cv::inRange(buffer2, lowerBound, upperBound, buffer1);

  // Perform Canny on the thresholded lightness image
  cv::Canny(buffer1, edges, lowerThreshold, upperThreshold, 3);

  return std::move(edges);
}

cv::Mat MitchFilter::filter(const cv::Mat &to_filter) {
  // Blur the image to reduce noise
  cv::GaussianBlur(to_filter, hsv, cv::Size(3, 3), 0);

  // Convert to HSV
  cv::cvtColor(hsv, hsv, cv::COLOR_RGB2HSV);

  // Do the Mitch method, subtract the saturation channel from the lightness
  // channel
  cv::split(hsv, split_hsv);
  cv::subtract(split_hsv[2], split_hsv[3], buffer);
  cv::multiply(buffer, 2, buffer);
  cv::subtract(buffer, 255, buffer);

  cv::imshow(buffer);

  // Perform Canny on mitch filtered img
  cv::Canny(buffer, edges, 0.0, 255.0);

  cv::imshow(edges);

  cv::waitKey(1);

  return std::move(edges);
}

}  // namespace umigv
