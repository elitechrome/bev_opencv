#pragma once
#include "camera_model.hpp"
#include "ipm_strategy.hpp"
#include <vector>

namespace bev {

class IPMHomography : public IPMStrategy {
public:
  /**
   * @brief Initialize homography and precompute combined distortion correction
   * maps
   * @param params Camera parameters (K, D, fisheye flag)
   * @param src_points 4 points in source image (undistorted)
   * @param dst_points 4 points in target BEV image
   * @param target_size Output image size
   */
  void init(const CameraParameters &params,
            const std::vector<cv::Point2f> &src_points,
            const std::vector<cv::Point2f> &dst_points,
            const cv::Size &target_size);

  void process(const cv::Mat &input, cv::Mat &output) override;

private:
  cv::Mat map_x_, map_y_;
  cv::Size target_size_;
};

} // namespace bev
