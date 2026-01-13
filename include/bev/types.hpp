#pragma once

#include <opencv2/opencv.hpp>
#include <string>

namespace bev {

struct CameraIntrinsics {
  cv::Mat K;
  cv::Mat D;
  std::string distortion_model; // "plumb_bob" or "fisheye"
};

struct SystemConfig {
  // Extrinsics (4x4 Transformation Matrices)
  cv::Mat T_robot_cam;     // Robot -> Camera
  cv::Mat T_robot_virtual; // Robot -> Virtual Ortho View

  CameraIntrinsics intrinsics;

  // Target Spec
  float meter_per_px = 0.02f;
  cv::Size target_resolution; // Derived from coverage area or specified

  // Optional Demo Input
  std::string input_image_path;

  // Live Stream
  std::string input_source = "image"; // "image", "stream"
  int camera_id = 0;
};

struct GeometricParams {
  // Poses are 4x4 matrices (cv::Mat, CV_64F)
  cv::Mat T_robot_camera; // Extrinsics: Robot -> Camera
  cv::Mat T_robot_bev;    // Virtual Camera Pose: Robot -> Virtual Camera

  // Virtual Camera Intrinsics (Orthographic)
  double target_res_width;  // Width in pixels
  double target_res_height; // Height in pixels
  double meter_per_px;      // Scale in meters/pixel
};

} // namespace bev
