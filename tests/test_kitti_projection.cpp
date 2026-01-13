#include "bev/camera_model.hpp"
#include "bev/ipm_geometric.hpp"
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

// Define a simple assertion macro
#define ASSERT_NEAR(val1, val2, abs_error)                                     \
  if (std::abs((val1) - (val2)) > (abs_error)) {                               \
    std::cerr << "Assertion failed: " << #val1 << " (" << (val1)               \
              << ") != " << #val2 << " (" << (val2)                            \
              << "), difference: " << std::abs((val1) - (val2)) << std::endl;  \
    return 1;                                                                  \
  }

int main() {
  std::cout << "[Test] Starting KITTI Projection Test..." << std::endl;

  // 1. Load Parameters (Hardcoded to match config/kitti_config.yaml &
  // kitti_intrinsics.yaml) Avoid dependency on FileStorage for unit test
  // isolation if possible, but reading file is fine.

  std::string config_path = "config/kitti_intrinsics.yaml";
  bev::CameraModel cam_model;
  if (!cam_model.loadFromYaml(config_path)) {
    std::cerr << "[Test] Failed to load " << config_path << std::endl;
    return 1;
  }

  // Geometric Params
  bev::GeometricParams geo_params;
  // T_robot_cam (Height 1.65m, Look Forward)
  // Rot: X_c->-Y_r, Y_c->-Z_r, Z_c->X_r
  geo_params.T_robot_camera = (cv::Mat_<double>(4, 4) << 0, 0, 1, 0, -1, 0, 0,
                               0, 0, -1, 0, 1.65, 0, 0, 0, 1);

  // T_robot_bev (Center at X=25m, Y=0)
  geo_params.T_robot_bev = (cv::Mat_<double>(4, 4) << 1, 0, 0, 25.0, 0, 1, 0,
                            0.0, 0, 0, 1, 0.0, 0, 0, 0, 1);

  geo_params.target_res_width = 600;
  geo_params.target_res_height = 1000;
  geo_params.meter_per_px = 0.05; // 0.05 m/px = 20 px/m

  bev::IPMGeometric ipm_subsystem;
  ipm_subsystem.init(cam_model.getParams(), geo_params);

  // 2. Define Test Points on Ground (Z=0 in Robot Frame)
  // Points along the center lane ahead
  std::vector<cv::Point3f> ground_points = {
      cv::Point3f(15.0, 0.0, 0.0),  // 15m ahead
      cv::Point3f(20.0, 0.0, 0.0),  // 20m ahead
      cv::Point3f(30.0, 0.0, 0.0),  // 30m ahead
      cv::Point3f(20.0, -2.0, 0.0), // 20m ahead, 2m left
      cv::Point3f(20.0, 2.0, 0.0)   // 20m ahead, 2m right
  };

  // 3. Project Ground -> Image (Forward Projection) using Camera Model
  // P_cam = T_cam_robot * P_robot
  cv::Mat T_cam_robot = geo_params.T_robot_camera.inv();

  std::cout << "[Test] Verifying Round Trip (Ground -> Image -> BEV -> Ground)"
            << std::endl;
  // Actually, we don't have Inverse IPM (BEV -> Ground) implemented explicitly.
  // But we map Image -> BEV.
  // So we can check:
  // P_ground -> P_image (Calculated here)
  // P_image -> P_bev (via IPM Process / Map lookup)
  // P_bev -> P_ground_recalc (via BEV Geometry)
  // Assert P_ground ~= P_ground_recalc

  const float ground_resolution =
      geo_params.meter_per_px; // meters/pixel = 0.05
  const int bev_w = geo_params.target_res_width;
  const int bev_h = geo_params.target_res_height;

  for (const auto &P_g : ground_points) {
    // Transform to Camera Frame
    cv::Mat P_g_vec = (cv::Mat_<double>(4, 1) << P_g.x, P_g.y, P_g.z, 1.0);
    cv::Mat P_c_vec = T_cam_robot * P_g_vec;

    cv::Point3f P_c(P_c_vec.at<double>(0), P_c_vec.at<double>(1),
                    P_c_vec.at<double>(2));

    // Project to Image
    std::vector<cv::Point3f> obj_pts = {P_c};
    std::vector<cv::Point2f> img_pts;
    cam_model.projectPoints(obj_pts, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
                            img_pts);

    cv::Point2f uv = img_pts[0];

    // Check if in image bounds
    if (uv.x < 0 || uv.x >= 1242 || uv.y < 0 || uv.y >= 375) {
      std::cout << "Skipping point " << P_g << " (Outside image: " << uv << ")"
                << std::endl;
      continue;
    }

    // 4. Map (u,v) -> BEV (u', v') via IPM Map
    // We need to access the map directly or simulate the mapping.
    // IPMGeometric::process does remap.
    // The maps are stored privately.
    // BUT we can re-calculate the expected BEV coordinate easily from P_ground
    // for validation.

    // Expected BEV Coordinate
    // P_bev_metric = T_bev_robot * P_ground? No T_robot_bev is P_rob = T *
    // P_bev_metric So P_bev_metric = T_robot_bev_inv * P_ground
    cv::Mat T_bev_robot = geo_params.T_robot_bev.inv();
    cv::Mat P_bm_vec = T_bev_robot * P_g_vec;

    double x_bm = P_bm_vec.at<double>(0);
    double y_bm = P_bm_vec.at<double>(1);

    // Convert metric to pixels
    // Assumes P_bev_metric (0,0) is center of image?
    // Logic in ipm_geometric:
    // x_sens = (u - w/2) * res   => u = x_sens/res + w/2
    // y_sens = (v - h/2) * res   => v = y_sens/res + h/2

    double u_bev = x_bm * (1.0 / geo_params.meter_per_px) + bev_w / 2.0;
    double v_bev = y_bm * (1.0 / geo_params.meter_per_px) + bev_h / 2.0;

    std::cout << "Point Ground: " << P_g << " -> Image: " << uv
              << " -> Expected BEV: (" << u_bev << ", " << v_bev << ")"
              << std::endl;

    // Now, how to verify the actual IPM map matches this?
    // We can create a 1-pixel image at UV, warp it, and see where it lands?
    // Or make `IPMGeometric` expose a `projectGroundToBEV` or
    // `projectImageToBEV` function? But `IPMGeometric` only precomputes
    // `map_x`, `map_y` for `remap`. `remap(bev_grid) -> input_image_pixels`. So
    // `map_x(v_bev, u_bev)` should equal `uv.x`. `map_y(v_bev, u_bev)` should
    // equal `uv.y`.

    // We need access to map_x_, map_y_ from IPMGeometric.
    // Or we just trust our math above matches the code in ipm_geometric.cpp
    // (it's the same logic inverted).

    // Let's use a "Black Box" test:
    // Create an input image that is black, set pixel (Round(uv)) to White.
    // Run process().
    // Check finding the white pixel in BEV image around (Round(u_bev),
    // Round(v_bev)).

    cv::Mat input = cv::Mat::zeros(375, 1242, CV_8UC1);
    cv::circle(input, uv, 2, cv::Scalar(255), -1); // Draw small dot

    cv::Mat output;
    ipm_subsystem.process(input, output);

    // Check output intensity at Expected BEV
    int u_b = std::round(u_bev);
    int v_b = std::round(v_bev);

    if (u_b >= 0 && u_b < bev_w && v_b >= 0 && v_b < bev_h) {
      if (P_g == ground_points[0]) { // Print info for first point
        std::cout << "DEBUG: Output Size: " << output.rows << "x" << output.cols
                  << " Type: " << output.type() << std::endl;
        std::cout << "DEBUG: Accessing at (" << v_b << ", " << u_b << ")"
                  << std::endl;
      }
      uchar val = 0;
      try {
        val = output.at<uchar>(v_b, u_b);
      } catch (const cv::Exception &e) {
        std::cerr << "OpenCV Error accessing pixel: " << e.what() << std::endl;
        return 1;
      }
      std::cout << "  BEV Pixel value at predicted location: " << (int)val
                << std::endl;
      if (val < 100) {
        // Search neighborhood
        bool found = false;
        for (int dy = -5; dy <= 5; dy++) {
          for (int dx = -5; dx <= 5; dx++) {
            int ny = v_b + dy;
            int nx = u_b + dx;
            if (ny >= 0 && ny < bev_h && nx >= 0 && nx < bev_w) {
              if (output.at<uchar>(ny, nx) > 100) {
                found = true;
                std::cout << "  Found nearby match at offset " << dx << ","
                          << dy << std::endl;
              }
            }
          }
        }
        if (!found) {
          std::cerr << "  FAILED: Point lost in transformation." << std::endl;
          return 1;
        }
      }
    } else {
      std::cout << "  BEV Point out of bounds." << std::endl;
    }
  }

  std::cout << "[Test] Success." << std::endl;
  return 0;
}
