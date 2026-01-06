#pragma once
#include <opencv2/opencv.hpp>

namespace bev {

struct IPMParams {
  // Common
  cv::Size target_size;

  // For Geometric
  cv::Mat map_x, map_y;

  // For Homography
  cv::Mat H;
};

class IPMStrategy {
public:
  virtual ~IPMStrategy() = default;

  /**
   * @brief Process input image to generate BEV image
   * @param input Raw or Undistorted source image
   * @param output Output BEV image
   */
  virtual void process(const cv::Mat &input, cv::Mat &output) = 0;
};

} // namespace bev
