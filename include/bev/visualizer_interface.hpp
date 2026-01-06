#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace bev {

class VisualizerInterface {
public:
  virtual ~VisualizerInterface() = default;

  virtual void init() = 0;

  // Setup methods
  virtual void setCameraIntrinsics(const cv::Mat &K,
                                   const cv::Size &img_size) = 0;
  virtual void setBEVGeometry(float width_m, float height_m, float res_m_px,
                              const cv::Mat &T_robot_bev) = 0;

  virtual void updateCommon(const cv::Mat &input_img, const cv::Mat &bev_img,
                            const cv::Mat &undistorted_img) = 0;

  // Geometric specific visualization
  virtual void setCameraPose(const cv::Mat &T_robot_cam) = 0;

  // Homography specific visualization
  virtual void setPointPairs(const std::vector<cv::Point2f> &src,
                             const std::vector<cv::Point2f> &dst) = 0;

  virtual void spinOnce() = 0;
};

} // namespace bev
