#include "bev/camera_model.hpp"
#include <iostream>

namespace bev {

bool CameraModel::loadFromYaml(const std::string &yaml_path) {
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "[CameraModel] Failed to open " << yaml_path << std::endl;
    return false;
  }

  cv::Mat K, D;
  fs["camera_matrix"] >> K;
  fs["dist_coeffs"] >> D;

  cv::Size image_size;
  if (!fs["image_width"].empty() && !fs["image_height"].empty()) {
    int w, h;
    fs["image_width"] >> w;
    fs["image_height"] >> h;
    image_size = cv::Size(w, h);
  }

  bool is_fisheye = false;
  std::string distortion_model;
  if (!fs["distortion_model"].empty()) {
    fs["distortion_model"] >> distortion_model;
    if (distortion_model == "fisheye") {
      is_fisheye = true;
    }
  }

  if (K.empty() || D.empty()) {
    std::cerr << "[CameraModel] Failed to load K or D from " << yaml_path
              << std::endl;
    return false;
  }

  setIntrinsics(K, D, image_size, is_fisheye);
  std::cout << "[CameraModel] Loaded intrinsics from " << yaml_path
            << " (Model: " << (is_fisheye ? "Fisheye" : "Standard") << ")"
            << std::endl;
  return true;
}

void CameraModel::setIntrinsics(const cv::Mat &K, const cv::Mat &D,
                                const cv::Size &size, bool is_fisheye) {
  params_.K = K.clone();
  params_.D = D.clone();
  params_.image_size = size;
  params_.is_fisheye = is_fisheye;

  if (is_fisheye) {
    cv::fisheye::initUndistortRectifyMap(params_.K, params_.D, cv::Mat(),
                                         params_.K, params_.image_size,
                                         CV_32FC1, map1_, map2_);
  } else {
    cv::initUndistortRectifyMap(params_.K, params_.D, cv::Mat(), params_.K,
                                params_.image_size, CV_32FC1, map1_, map2_);
  }
  init_done_ = true;
}

cv::Mat CameraModel::undistort(const cv::Mat &input) const {
  if (!init_done_)
    return input;
  cv::Mat output;
  cv::remap(input, output, map1_, map2_, cv::INTER_LINEAR);
  return output;
}

void CameraModel::projectPoints(const std::vector<cv::Point3f> &obj_pts,
                                const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                                std::vector<cv::Point2f> &img_pts) const {
  if (obj_pts.empty())
    return;

  if (params_.is_fisheye) {
    cv::fisheye::projectPoints(obj_pts, img_pts, rvec, tvec, params_.K,
                               params_.D);
  } else {
    cv::projectPoints(obj_pts, rvec, tvec, params_.K, params_.D, img_pts);
  }
}

} // namespace bev
