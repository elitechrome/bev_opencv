#pragma once

#include "bev/types.hpp"
#include "bev/vision_pipeline.hpp"
#include <string>
#include <vector>

namespace bev {

class SequentialProcessor : public IVisionPipeline {
public:
  explicit SequentialProcessor(const SystemConfig &config);

  [[nodiscard]] cv::Mat process(const cv::Mat &raw_image) override;
  void printStats() const override;

private:
  SystemConfig config_;
  cv::Mat H_; // Homography matrix
  GeometricParams geo_params_;

  // Profiling
  mutable std::vector<std::pair<std::string, double>> stats_;

  void computeHomography();
};

} // namespace bev
