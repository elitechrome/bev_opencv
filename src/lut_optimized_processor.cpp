#include "bev/lut_optimized_processor.hpp"
#include "bev/profiler.hpp"
#include <iostream>

namespace bev {

namespace {
// Mock NPU inference (duplicated for now, or could move to shared util)
cv::Mat runMockInference(const cv::Mat &input) {
  cv::Mat gray, mask;
  cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, mask, 100, 255, cv::THRESH_BINARY_INV);
  return mask;
}
} // namespace

LutOptimizedProcessor::LutOptimizedProcessor(const SystemConfig &config)
    : config_(config) {
  // Populate GeometricParams from SystemConfig for reuse
  // We assume T_robot_camera provided is the full transform and T_robot_bev is
  // T_robot_virtual
  geo_params_.T_robot_camera = config_.T_robot_cam;
  geo_params_.T_robot_bev = config_.T_robot_virtual;
  geo_params_.target_res_width = config_.target_resolution.width;
  geo_params_.target_res_height = config_.target_resolution.height;
  geo_params_.meter_per_px = config_.meter_per_px;

  // Defer initLUT until we know the input image size in process()
}

void LutOptimizedProcessor::initLUT(const cv::Size &size) {
  std::cout << "[LUT] Initializing LUT for input size: " << size << std::endl;
  input_size_ = size;

  // Target Resolution
  int vw = (int)geo_params_.target_res_width;
  int vh = (int)geo_params_.target_res_height;
  double res_m = geo_params_.meter_per_px; // meters per pixel

  map_x_ = cv::Mat(vh, vw, CV_32F);
  map_y_ = cv::Mat(vh, vw, CV_32F);

  float cx_v = vw / 2.0f;
  float cy_v = vh / 2.0f;

  // 1. Prepare Ray Casting Constants
  // Virtual Camera (Orthographic) logic from IPMGeometric
  // T_robot_bev is Virtual -> Robot (Pose of Virtual Camera)
  // WE NEED: Virtual -> Robot for ray casting origin.
  cv::Mat T_virt_rob_pose = geo_params_.T_robot_bev;

  // Direction: Z-axis of Virtual Camera (Forward view)
  // D_virt = (0, 0, 1, 0)
  cv::Mat D_virt = (cv::Mat_<double>(4, 1) << 0, 0, 1, 0);
  cv::Mat D_rob = T_virt_rob_pose *
                  D_virt; // Transform Vector only (rotation part effectively)
  // Ideally T_virt_rob_pose is 4x4, D_virt is 4x1 (vector).
  // Note: if T_virt_rob_pose includes translation, 4x4 * [0,0,1,0] uses
  // rotation only which is correct for direction.

  cv::Vec3d Dir(D_rob.at<double>(0), D_rob.at<double>(1), D_rob.at<double>(2));

  if (std::abs(Dir[2]) < 1e-6) {
    // Parallel to ground
    map_x_.setTo(-1);
    map_y_.setTo(-1);
    return;
  }

  // Optimize: Pre-calculate scaling/offset for virtual grid
  std::vector<cv::Point3f> P_virt_vec;
  P_virt_vec.reserve(vw * vh);

  for (int y = 0; y < vh; ++y) {
    for (int x = 0; x < vw; ++x) {
      // center centered coordinates, similar to IPMGeometric
      double x_sens = (x - vw / 2.0) * res_m;
      double y_sens = (y - vh / 2.0) * res_m;
      P_virt_vec.emplace_back(x_sens, y_sens, 0.0f);
    }
  }

  // 2. Batch Ray Casting
  // We need P_robot for each P_virt. P_rob_origin = T_rv * P_virt
  std::vector<cv::Point3f> P_rob_origin_vec;
  cv::perspectiveTransform(P_virt_vec, P_rob_origin_vec, T_virt_rob_pose);

  // Compute Intersection t = -P0.z / Dir.z
  // P_ground = P0 + t * Dir
  // Since Dir is constant, we can vectorize this easily.

  std::vector<cv::Point3f> P_ground_vec;
  P_ground_vec.reserve(vw * vh);

  double inv_dir_z = 1.0 / Dir[2];
  double dir_x = Dir[0];
  double dir_y = Dir[1];
  double dir_z = Dir[2];

  for (const auto &p0 : P_rob_origin_vec) {
    double t = -p0.z * inv_dir_z;
    P_ground_vec.emplace_back(p0.x + t * dir_x, p0.y + t * dir_y,
                              p0.z + t * dir_z);
  }

  // 3. Transform to Optical Frame (Camera Frame)
  // P_cam = T_rc * P_ground => T_opt_robot * P_ground
  // geo_params_.T_robot_camera is Transform Robot->Camera (Extrinsics).
  // We want Ground(Robot) -> Camera. So we use the Inverse of Robot->Camera?
  // config_.T_robot_cam IS the extrinsics. Usually T_camera_robot is what
  // projects Robot points to Camera. BUT the config says "T_robot_cam" -
  // ambiguous common issue. Let's assume T_robot_cam means Pose of Camera IN
  // Robot Frame (T_robot_camera). Then needed transform for Points is
  // T_camera_robot = T_robot_camera.inv() Wait, existing code used
  // `cv::perspectiveTransform(P_ground_vec, P_cam_vec, config_.T_robot_cam);`
  // If `T_robot_cam` was used to transform Ground points (Robot frame) to
  // Camera frame, then T_robot_cam MUST BE the coordinate transform matrix
  // (T_cam_robot). However, in IPMGeometric, we saw: cv::Mat T_robot_opt =
  // geo_params_.T_robot_camera; cv::Mat T_opt_robot = T_robot_opt.inv();
  // cv::Mat P_opt = T_opt_robot * P_g_mat;
  // This implies geo_params_.T_robot_camera is the POSE (Robot->Camera).
  // So we should invert it to get the transform matrix for points.

  cv::Mat T_cam_robot = geo_params_.T_robot_camera.inv();
  std::vector<cv::Point3f> P_cam_vec;
  cv::perspectiveTransform(P_ground_vec, P_cam_vec, T_cam_robot);

  // 4. Batch Project Points
  std::vector<cv::Point2f> imagePoints;
  cv::Vec3d zero_rvec(0, 0, 0);
  cv::Vec3d zero_tvec(0, 0, 0);

  if (config_.intrinsics.distortion_model == "fisheye") {
    cv::fisheye::projectPoints(P_cam_vec, imagePoints, zero_rvec, zero_tvec,
                               config_.intrinsics.K, config_.intrinsics.D);
  } else {
    cv::projectPoints(P_cam_vec, zero_rvec, zero_tvec, config_.intrinsics.K,
                      config_.intrinsics.D, imagePoints);
  }

  // 5. Fill Maps with Post-Processing
  // 5. Fill Maps with Post-Processing
  // We map from Source (Input Image Size) -> BEV
  // If we are using NPU input size (384x128), we need to handle that scaling in
  // 'process', NOT HERE. The LUT should map BEV pixels -> Original Image Pixels
  // (UV). Then in `process`, if we resized the input, we must adjust. BUT the
  // architecture in `SequentialProcessor` and `process` here does: Input ->
  // Resize(NPU) -> Inference -> Mask -> Resize(Input) -> Remap(BEV) OR Input ->
  // Remap(BEV) ??
  //
  // Original `SequentialProcessor`:
  // Input -> Resize 384x128 -> Inference -> Mask(384x128) -> Resize to
  // Original(Full) -> Undistort -> Warp
  //
  // Original `LutOptimizedProcessor`:
  // Input -> Resize 384x128 -> Inference -> Mask(384x128) -> Remap(BEV, using
  // Lookups into Mask)
  //
  // So the Source image for Remap IS the Mask(384x128) !!
  //
  // So validation of LUT coordinates must be against 384x128 (NPU Output Size).
  // This means the LUT depends on NPU Output Size, not the RAW Input Size.
  //
  // WAIT. If the LUT is mapping BEV -> Mask(384x128), then `initLUT` doesn't
  // depend on raw input resolution!
  //
  // However, the `process` function originally did:
  // `cv::resize(roi, input_npu, cv::Size(384, 128));`
  // `cv::Mat mask = runMockInference(input_npu);`
  // `cv::remap(mask, final_map, map_x_, map_y_ ...)`
  //
  // The `imagePoints` we computed above are in the ORIGINAL CAMERA FRAME
  // (Undistorted/Distorted Pinhole pixels). The `mask` is a cropped & resized
  // version of the original image!
  //
  // So we have a coordinate transform chain:
  // BEV (x,y) -> [RayCast] -> Camera (u_raw, v_raw) -> [Distortion] ->
  // Distorted Image (u_dist, v_dist)
  //
  // The 'mask' corresponds to:
  // 1. Crop (Bottom Half)
  // 2. Resize to 384x128
  //
  // We need to map (u_dist, v_dist) -> (u_mask, v_mask).
  //
  // u_mask = (u_dist - crop_x_start) * (mask_w / crop_w)
  // v_mask = (v_dist - crop_y_start) * (mask_h / crop_h)
  //
  // Here, crop is Bottom Half.
  // crop_x_start = 0, crop_w = input_w
  // crop_y_start = input_h / 2, crop_h = input_h / 2
  // mask_w = 384, mask_h = 128
  //
  // Therefore, we DO need the input resolution (input_w, input_h) to compute
  // the scaling factors to the fixed NPU size.

  double input_w = size.width;
  double input_h = size.height;
  double roi_y_start = input_h / 2.0;
  // Scale factors from Original (Cropped) -> NPU
  double scale_x = 384.0 / input_w;
  double scale_y = 128.0 / (input_h / 2.0);

  float *mx_ptr = map_x_.ptr<float>();
  float *my_ptr = map_y_.ptr<float>();

  for (size_t i = 0; i < imagePoints.size(); ++i) {
    // Check Depth implicitly or explicitly
    if (P_cam_vec[i].z <= 1e-3) {
      mx_ptr[i] = -1.f;
      my_ptr[i] = -1.f;
      continue;
    }

    double u_raw = imagePoints[i].x;
    double v_raw = imagePoints[i].y;

    if (u_raw >= 0 && u_raw < input_w && v_raw >= roi_y_start &&
        v_raw < input_h) {
      mx_ptr[i] = static_cast<float>(u_raw * scale_x);
      my_ptr[i] = static_cast<float>((v_raw - roi_y_start) * scale_y);
    } else {
      mx_ptr[i] = -1.f;
      my_ptr[i] = -1.f;
    }
  }
}

cv::Mat LutOptimizedProcessor::process(const cv::Mat &raw_image) {
  ScopedTimer timer("LutOptimizedProcessor", stats_);

  // Lazy Init if resolution changes
  if (raw_image.size() != input_size_) {
    initLUT(raw_image.size());
  }

  // 1. Preprocess
  int h = raw_image.rows;
  int w = raw_image.cols;
  cv::Rect roi_rect(0, h / 2, w, h / 2);
  cv::Mat roi = raw_image(roi_rect);
  cv::Mat input_npu;
  cv::resize(roi, input_npu, cv::Size(384, 128));

  // 2. Inference
  cv::Mat mask = runMockInference(input_npu);

  // 3. Post-process (Single Remap)
  cv::Mat final_map;
  // Note: mask is 384x128. map_x/y contains coordinates in this 384x128 space.
  // If map values are -1, remap handles it (borderValue).

  // Map usage: dst(x,y) = src(map_x(x,y), map_y(x,y))
  // Src is 'mask'.
  cv::remap(mask, final_map, map_x_, map_y_, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0));

  return final_map;
}

void LutOptimizedProcessor::printStats() const {
  std::cout << "=== LutOptimizedProcessor Stats ===" << std::endl;
  for (const auto &[name, time] : stats_) {
    std::cout << name << ": " << time << " ms" << std::endl;
  }
  stats_.clear();
}

} // namespace bev
