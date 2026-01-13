#pragma once

#include <opencv2/opencv.hpp>

namespace bev {

class IVisionPipeline {
public:
  virtual ~IVisionPipeline() = default;

  // Main processing method
  // Returns: The final top-down obstacle mask
  [[nodiscard]] virtual cv::Mat process(const cv::Mat &raw_image) = 0;

  // Performance metrics
  virtual void printStats() const = 0;
};

} // namespace bev
