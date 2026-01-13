#include "bev/ipm_geometric.hpp"
#include <iostream>
#include <opencv2/calib3d.hpp>

namespace bev {

void IPMGeometric::init(const CameraParameters &cam_params,
                        const GeometricParams &geo_params) {
  cam_params_ = cam_params;
  geo_params_ = geo_params;
  precomputeMaps();
}

void IPMGeometric::process(const cv::Mat &input, cv::Mat &output) {
  if (map_x_.empty() || map_y_.empty())
    return;
  cv::remap(input, output, map_x_, map_y_, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

void IPMGeometric::precomputeMaps() {
  std::cout << "[IPMGeometric] Precomputing maps..." << std::endl;

  int w = (int)geo_params_.target_res_width;
  int h = (int)geo_params_.target_res_height;

  map_x_ = cv::Mat(h, w, CV_32F);
  map_y_ = cv::Mat(h, w, CV_32F);

  // 1. Setup Transforms
  // T_robot_opt = T_robot_cam
  // We assume T_robot_camera provided in config IS the full transform from
  // Optical Frame to Robot Frame.
  cv::Mat T_robot_opt = geo_params_.T_robot_camera;
  cv::Mat T_opt_robot = T_robot_opt.inv();

  // Virtual Camera (Orthographic) logic
  double res_m = geo_params_.meter_per_px;

  // Pre-allocate pts for batch projection if we wanted optimization,
  // but nested loop is clearer for logic.

  // Direction: Z-axis of Virtual Camera (Forward view)
  // We assume Virtual Camera Look Direction defines the ray direction.
  // D_virt = (0, 0, 1, 0)
  cv::Mat D_virt = (cv::Mat_<double>(4, 1) << 0, 0, 1, 0);
  cv::Mat D_rob = geo_params_.T_robot_bev * D_virt;
  cv::Vec3d Dir(D_rob.at<double>(0), D_rob.at<double>(1), D_rob.at<double>(2));

  std::cout << "[IPMGeometric] Precomputing maps w=" << w << " h=" << h
            << std::endl;

  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u) {
      // center centered coordinates
      double x_sens = (u - w / 2.0) * res_m;
      double y_sens = (v - h / 2.0) * res_m;

      // P_virt on sensor plane (z=0)
      cv::Mat P_virt = (cv::Mat_<double>(4, 1) << x_sens, y_sens, 0, 1);
      cv::Mat P_rob_origin = geo_params_.T_robot_bev * P_virt;
      cv::Vec3d P0(P_rob_origin.at<double>(0), P_rob_origin.at<double>(1),
                   P_rob_origin.at<double>(2));

      // Intersect with Z=0 in Robot Frame
      // P = P0 + t * Dir
      // P.z = 0 => P0.z + t * Dir.z = 0

      if (std::abs(Dir[2]) < 1e-6) {
        map_x_.at<float>(v, u) = -1;
        map_y_.at<float>(v, u) = -1;
        continue;
      }

      double t = -P0[2] / Dir[2];
      cv::Vec3d P_ground = P0 + t * Dir;

      // Transform to Optical Frame
      cv::Mat P_g_mat = (cv::Mat_<double>(4, 1) << P_ground[0], P_ground[1],
                         P_ground[2], 1.0);
      cv::Mat P_opt = T_opt_robot * P_g_mat;

      double Xc = P_opt.at<double>(0);
      double Yc = P_opt.at<double>(1);
      double Zc = P_opt.at<double>(2);

      if (Zc <= 0) {
        map_x_.at<float>(v, u) = -1;
        map_y_.at<float>(v, u) = -1;
        continue;
      }

      // Project
      std::vector<cv::Point3f> obj_pts = {cv::Point3f(Xc, Yc, Zc)};
      std::vector<cv::Point2f> img_pts;
      cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 0);

      if (cam_params_.is_fisheye) {
        cv::fisheye::projectPoints(obj_pts, img_pts, rvec, tvec, cam_params_.K,
                                   cam_params_.D);
      } else {
        cv::projectPoints(obj_pts, rvec, tvec, cam_params_.K, cam_params_.D,
                          img_pts);
      }

      map_x_.at<float>(v, u) = img_pts[0].x;
      map_y_.at<float>(v, u) = img_pts[0].y;
    }
  }

  std::cout << "[IPMGeometric] Maps generated." << std::endl;
}

std::vector<cv::Point3f> IPMGeometric::getFrustumGroundPoints() const {
  return {};
}

} // namespace bev
