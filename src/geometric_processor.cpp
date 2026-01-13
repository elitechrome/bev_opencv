#include "bev/geometric_processor.hpp"
#include "bev/profiler.hpp"
#include <iostream>

namespace bev {

GeometricProcessor::GeometricProcessor(const SystemConfig &config)
    : config_(config) {
  geo_params_.T_robot_camera = config_.T_robot_cam;
  geo_params_.T_robot_bev = config_.T_robot_virtual;
  geo_params_.target_res_width = config_.target_resolution.width;
  geo_params_.target_res_height = config_.target_resolution.height;
  geo_params_.meter_per_px = config_.meter_per_px;
}

void GeometricProcessor::init(const cv::Size &size) {
  input_size_ = size;
  CameraParameters cam_params;
  cam_params.K = config_.intrinsics.K;
  cam_params.D = config_.intrinsics.D;
  cam_params.image_size = size;
  cam_params.is_fisheye = (config_.intrinsics.distortion_model == "fisheye");

  ipm_.init(cam_params, geo_params_);
}

cv::Mat GeometricProcessor::process(const cv::Mat &raw_image) {
  ScopedTimer timer("GeometricProcessor: Total", stats_);

  if (raw_image.size() != input_size_) {
    init(raw_image.size());
  }

  cv::Mat output;
  ipm_.process(raw_image, output);
  return output;
}

void GeometricProcessor::printStats() const {
  std::cout << "=== GeometricProcessor Stats ===" << std::endl;
  for (const auto &[name, time] : stats_) {
    std::cout << name << ": " << time << " ms" << std::endl;
  }
  stats_.clear();
}

} // namespace bev
