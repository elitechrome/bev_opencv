#pragma once

#include "bev/ipm_geometric.hpp"
#include "bev/types.hpp"
#include "bev/vision_pipeline.hpp"

namespace bev {

class GeometricProcessor : public IVisionPipeline {
public:
  explicit GeometricProcessor(const SystemConfig &config);

  [[nodiscard]] cv::Mat process(const cv::Mat &raw_image) override;
  void printStats() const override;

private:
  SystemConfig config_;
  IPMGeometric ipm_;

  cv::Size input_size_;
  GeometricParams geo_params_;

  mutable std::vector<std::pair<std::string, double>> stats_;

  void init(const cv::Size &size);
};

} // namespace bev
