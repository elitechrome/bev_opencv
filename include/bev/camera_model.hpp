#pragma once
#include <opencv2/opencv.hpp>
#include <string>

namespace bev {

struct CameraParameters {
  cv::Mat K;
  cv::Mat D;
  cv::Size image_size = cv::Size(0, 0);
  bool is_fisheye = false; // [NEW] Fisheye flag
};

class CameraModel {
public:
  bool loadFromYaml(const std::string &yaml_path);
  void setIntrinsics(const cv::Mat &K, const cv::Mat &D, const cv::Size &size,
                     bool is_fisheye = false);

  cv::Mat undistort(const cv::Mat &input) const;

  /**
   * @brief Project 3D points to 2D image points handling both standard and
   * fisheye models.
   */
  void projectPoints(const std::vector<cv::Point3f> &obj_pts,
                     const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                     std::vector<cv::Point2f> &img_pts) const;

  const CameraParameters &getParams() const { return params_; }

private:
  CameraParameters params_;
  cv::Mat map1_, map2_;
  bool init_done_ = false;
};

} // namespace bev
