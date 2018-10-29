#ifndef UMIGV_OBSTACLE_DETECTOR_CV_H
#define UMIGV_OBSTACLE_DETECTOR_CV_H

#include <opencv2/core/core.hpp>
#include <vector>

namespace umigv {

class CannyFilter {
 public:
  CannyFilter() = default;
  CannyFilter(const cv::Scalar &lower_bound, const cv::Scalar &upper_bound,
              double lower_threshold, double upper_threshold);
  cv::Mat filter(const cv::Mat &to_filter);

 private:
  cv::Mat buffer1;
  cv::Mat buffer2;
  cv::Mat edges;

  // HSV bounds of color filter
  cv::Scalar lowerBound = cv::Scalar(0, 0, 200);
  cv::Scalar upperBound = cv::Scalar(255, 30, 255);
  
   // Canny hysteresis thresholds
  double lowerThreshold = 0.0;
  double upperThreshold = 255.0;
};

class MitchFilter {
  public:
    MitchFilter() = default;
    cv::Mat filter(const cv::Mat &to_filter);

  private:
    cv::Mat hsv;
    cv::Mat edges;
    std::vector<cv::Mat> split_hsv;
    cv::Mat buffer;
}

} // namespace umigv

#endif
