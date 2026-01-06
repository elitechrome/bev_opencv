#pragma once
#include "camera_model.hpp"
#include "ipm_strategy.hpp"

namespace bev {

struct GeometricParams {
  // Poses are 4x4 matrices (cv::Mat, CV_64F)
  cv::Mat T_robot_camera; // Extrinsics: Robot -> Camera
  cv::Mat T_robot_bev;    // Virtual Camera Pose: Robot -> Virtual Camera

  // Virtual Camera Intrinsics (Orthographic)
  double target_res_width;  // Width in pixels
  double target_res_height; // Height in pixels
  double scale_px_per_mm;   // Scale
};

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
