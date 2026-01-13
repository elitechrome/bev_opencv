#pragma once

#include "bev/types.hpp"
#include "bev/vision_pipeline.hpp"
#include <string>
#include <vector>

namespace bev {

class LutOptimizedProcessor : public IVisionPipeline {
public:
  explicit LutOptimizedProcessor(const SystemConfig &config);

  [[nodiscard]] cv::Mat process(const cv::Mat &raw_image) override;
  void printStats() const override;

private:
  SystemConfig config_;

  // LUTs for remap
  cv::Mat map_x_;
  cv::Mat map_y_;

  // State
  cv::Size input_size_;
  GeometricParams geo_params_;

  // Profiling
  mutable std::vector<std::pair<std::string, double>> stats_;

  void initLUT(const cv::Size &size);
};

} // namespace bev
