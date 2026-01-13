#include "bev/sequential_processor.hpp"
#include "bev/profiler.hpp"
#include <iostream>

namespace bev {

namespace {
// Mock NPU inference: simple thresholding or dummy mask
// Input: 384x128 RGB
// Output: 384x128 Single Channel Mask (0 or 255)
const int kMockNpuWidth = 384;
const int kMockNpuHeight = 128;

cv::Mat runMockInference(const cv::Mat &input) {
  // Pass through original image to visualize RGB BEV
  return input.clone();
}
} // namespace

SequentialProcessor::SequentialProcessor(const SystemConfig &config)
    : config_(config) {
  // Populate GeometricParams
  geo_params_.T_robot_camera = config_.T_robot_cam;
  geo_params_.T_robot_bev = config_.T_robot_virtual;
  geo_params_.target_res_width = config_.target_resolution.width;
  geo_params_.target_res_height = config_.target_resolution.height;
  geo_params_.meter_per_px = config_.meter_per_px;

  computeHomography();
}

// Analytic Homography Computation
// H_bev2cam maps BEV pixel (u_v, v_v) -> Camera pixel (u_c, v_c)
// Chain: Pixel_BEV -> Metric_Virtual -> Metric_Robot -> Metric_Camera ->
// Pixel_Camera

// 1. Metric_Virtual (x_v, y_v) from Pixel_BEV (u_v, v_v)
void SequentialProcessor::computeHomography() {
  double w = geo_params_.target_res_width;
  double h = geo_params_.target_res_height;
  double res = geo_params_.meter_per_px;
  double cx_v = w / 2.0;
  double cy_v = h / 2.0;

  // M_scale: [u,v,1] -> [x_v, y_v, 1]
  cv::Mat M_scale = (cv::Mat_<double>(3, 3) << res, 0.0, -cx_v * res, 0.0, res,
                     -cy_v * res, 0.0, 0.0, 1.0);

  // 2. Metric_Robot from Metric_Virtual
  // P_r = T_robot_virt * P_v
  // We assume P_v has z=0.
  cv::Mat T_rob_virt = geo_params_.T_robot_bev;

  // 3. Metric_Camera from Metric_Robot
  // P_c = T_cam_rob * P_r
  cv::Mat T_cam_rob = geo_params_.T_robot_camera.inv();

  // Combine: T_cam_virt = T_cam_rob * T_rob_virt
  cv::Mat T_cam_virt = T_cam_rob * T_rob_virt;

  // Since P_v.z = 0, we can drop the 3rd column (index 2) of T_cam_virt
  // T_cam_virt is 4x4. We want 3x3 that acts on (x_v, y_v, 1) to produce (x_c,
  // y_c, z_c)
  cv::Mat T_cv_reduced = cv::Mat::zeros(3, 3, CV_64F);
  // Col 0 -> Col 0
  T_cam_virt.col(0).rowRange(0, 3).copyTo(T_cv_reduced.col(0));
  // Col 1 -> Col 1
  T_cam_virt.col(1).rowRange(0, 3).copyTo(T_cv_reduced.col(1));
  // Col 3 -> Col 2 (Translation)
  T_cam_virt.col(3).rowRange(0, 3).copyTo(T_cv_reduced.col(2));

  // 4. Pixel_Camera from Metric_Camera
  cv::Mat K = config_.intrinsics.K;

  // Final H
  H_ = K * T_cv_reduced * M_scale;

  std::cout << "[SeqH] Analytic Homography Computed." << std::endl;
  std::cout << "H:\n" << H_ << std::endl;
}

cv::Mat SequentialProcessor::process(const cv::Mat &raw_image) {
  ScopedTimer total_timer("Seq: Total", stats_);

  cv::Mat mask_full = cv::Mat::zeros(raw_image.size(), raw_image.type());
  int h = raw_image.rows;
  int w = raw_image.cols;
  cv::Rect roi_rect(0, h / 2, w, h / 2);

  // 1. Preprocess (Crop & Resize)
  cv::Mat input_npu;
  {
    ScopedTimer timer("Seq 1: Preprocess", stats_);
    cv::Mat roi = raw_image(roi_rect);
    cv::resize(roi, input_npu, cv::Size(kMockNpuWidth, kMockNpuHeight));
  }

  // 2. Inference
  cv::Mat mask_small;
  {
    ScopedTimer timer("Seq 2: Inference", stats_);
    mask_small = runMockInference(input_npu);
  }

  // 3. Resize mask back to full camera resolution
  {
    ScopedTimer timer("Seq 3: Postprocess Resize", stats_);
    cv::Mat mask_roi;
    cv::resize(mask_small, mask_roi, roi_rect.size(), 0, 0, cv::INTER_LINEAR);
    mask_roi.copyTo(mask_full(roi_rect));
  }

  // 4. Undistort
  cv::Mat undistorted;
  {
    ScopedTimer timer("Seq 4: Undistort", stats_);
    cv::undistort(mask_full, undistorted, config_.intrinsics.K,
                  config_.intrinsics.D);
  }

  // 5. Warp Perspective
  cv::Mat final_map;
  {
    ScopedTimer timer("Seq 5: Warp Perspective", stats_);
    if (H_.empty()) {
      return cv::Mat::zeros(config_.target_resolution, raw_image.type());
    }
    cv::warpPerspective(undistorted, final_map, H_, config_.target_resolution,
                        cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
                        cv::BORDER_CONSTANT, cv::Scalar(0));
  }

  return final_map;
}

void SequentialProcessor::printStats() const {
  std::cout << "=== SequentialProcessor Stats ===" << std::endl;
  for (const auto &[name, time] : stats_) {
    std::cout << name << ": " << time << " ms" << std::endl;
  }
  stats_.clear();
}

} // namespace bev
