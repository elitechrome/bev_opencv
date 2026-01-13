#pragma once
#include "camera_model.hpp"
#include "ipm_strategy.hpp"

#include "bev/types.hpp"

namespace bev {

class IPMGeometric : public IPMStrategy {
public:
  void init(const CameraParameters &cam_params,
            const GeometricParams &geo_params);
  void process(const cv::Mat &input, cv::Mat &output) override;

  // Helper to get ground plane points for visualization
  std::vector<cv::Point3f> getFrustumGroundPoints() const;

private:
  void precomputeMaps();

  CameraParameters cam_params_;
  GeometricParams geo_params_;
  cv::Mat map_x_, map_y_;
};

} // namespace bev
