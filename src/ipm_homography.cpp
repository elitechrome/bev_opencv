#include "bev/ipm_homography.hpp"
#include <iostream>
#include <opencv2/calib3d.hpp>

namespace bev {

void IPMHomography::init(const CameraParameters &params,
                         const std::vector<cv::Point2f> &src_points,
                         const std::vector<cv::Point2f> &dst_points,
                         const cv::Size &target_size) {
  target_size_ = target_size;
  cv::Mat H = cv::findHomography(src_points, dst_points);
  if (H.empty()) {
    std::cerr << "[IPMHomography] Failed to compute H!" << std::endl;
    return;
  }

  cv::Mat H_inv = H.inv();

  // Normalize Camera Matrix for projection direction
  // (We want to project normalized points to pixels with distortion, so we need
  // K)

  int w = target_size_.width;
  int h = target_size_.height;

  map_x_ = cv::Mat(h, w, CV_32F);
  map_y_ = cv::Mat(h, w, CV_32F);

  // Precompute combined map
  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u) {
      // 1. BEV Pixel -> Inverse Homography -> Undistorted Image Pixel
      // H_inv takes us from Target(u,v) -> Source(x', y')

      double x_src_undist, y_src_undist, w_src;
      // Manual mat mul for speed
      x_src_undist = H_inv.at<double>(0, 0) * u + H_inv.at<double>(0, 1) * v +
                     H_inv.at<double>(0, 2);
      y_src_undist = H_inv.at<double>(1, 0) * u + H_inv.at<double>(1, 1) * v +
                     H_inv.at<double>(1, 2);
      w_src = H_inv.at<double>(2, 0) * u + H_inv.at<double>(2, 1) * v +
              H_inv.at<double>(2, 2);

      if (std::abs(w_src) < 1e-6) {
        map_x_.at<float>(v, u) = -1;
        map_y_.at<float>(v, u) = -1;
        continue;
      }

      x_src_undist /= w_src;
      y_src_undist /= w_src;

      // 2. Undistorted Image Pixel -> Normalized 3D ray (Z=1)
      // p_undist = K * P_norm
      // P_norm = K_inv * p_undist

      double fx = params.K.at<double>(0, 0);
      double fy = params.K.at<double>(1, 1);
      double cx = params.K.at<double>(0, 2);
      double cy = params.K.at<double>(1, 2);

      double x_norm = (x_src_undist - cx) / fx;
      double y_norm = (y_src_undist - cy) / fy;

      // 3. Normalized 3D ray -> Distorted Image Pixel (Raw Image)
      // Use projectPoints to apply distortion

      std::vector<cv::Point3f> obj_pts = {cv::Point3f(x_norm, y_norm, 1.0)};
      std::vector<cv::Point2f> img_pts;
      cv::Vec3d rvec(0, 0, 0),
          tvec(0, 0, 0); // Identity pose relative to camera

      if (params.is_fisheye) {
        cv::fisheye::projectPoints(obj_pts, img_pts, rvec, tvec, params.K,
                                   params.D);
      } else {
        cv::projectPoints(obj_pts, rvec, tvec, params.K, params.D, img_pts);
      }

      map_x_.at<float>(v, u) = img_pts[0].x;
      map_y_.at<float>(v, u) = img_pts[0].y;
    }
  }
}

void IPMHomography::process(const cv::Mat &input, cv::Mat &output) {
  if (map_x_.empty() || map_y_.empty())
    return;
  // Remap uses the precomputed combined map
  cv::remap(input, output, map_x_, map_y_, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

} // namespace bev
