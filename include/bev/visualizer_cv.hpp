#pragma once
#include "visualizer_interface.hpp"

#ifdef BEV_ENABLE_CV_VIZ
#include <opencv2/viz.hpp>

namespace bev {

class VisualizerCV : public VisualizerInterface {
public:
  void init() override {
    viz_ = cv::viz::Viz3d("BEV Visualization");
    viz_.showWidget("World Frame",
                    cv::viz::WCoordinateSystem(1.0)); // World axes
    viz_.setWindowSize(cv::Size(1200, 900));

    // Add grid
    viz_.showWidget("Ground Grid",
                    cv::viz::WGrid(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 1),
                                   cv::Vec3d(0, 0, 1), 10.0, 10,
                                   cv::viz::Color::gray()));
  }

  void setCameraIntrinsics(const cv::Mat &K,
                           const cv::Size &img_size) override {
    K_ = K.clone();
    img_size_ = img_size;

    // 1. Camera Frustum
    double f_x = K.at<double>(0, 0);
    double f_y = K.at<double>(1, 1);
    double c_x = K.at<double>(0, 2);
    double c_y = K.at<double>(1, 2);

    // Create InputArray from K
    // OpenCV Viz WCameraPosition constructor takes Matx33d.
    cv::Matx33d K_matx = K_;
    viz_.showWidget("CameraFrustum", cv::viz::WCameraPosition(
                                         K_matx, 0.5, cv::viz::Color::white()));

    // Setup Plane dimensions for Input Image
    // Arbitrary scale for visibility (e.g. z=1.0)
    plane_z_ = 1.0;
    plane_w_ = plane_z_ * img_size.width / f_x;
    plane_h_ = plane_z_ * img_size.height / f_y; // Approx
  }

  void setBEVGeometry(float width_m, float height_m, float res_m_px,
                      const cv::Mat &T_robot_bev) override {
    bev_width_m_ = width_m;
    bev_height_m_ = height_m;
    if (!T_robot_bev.empty())
      T_robot_bev_ = T_robot_bev.clone();
  }

  void setCameraPose(const cv::Mat &T_robot_cam) override {
    if (T_robot_cam.empty())
      return;

    // Ensure T_robot_cam is 3x4 or 4x4, convert to Affine3d
    cv::Affine3d pose(T_robot_cam);
    viz_.setWidgetPose("CameraFrustum", pose);

    // Update Input Image Plane Pose (Camera Pose * Local Offset)
    // Wrap in try-catch because InputImagePlane might not be created yet
    // (it is created in updateCommon when first image arrives).
    try {
      cv::Affine3d plane_local =
          cv::Affine3d().translate(cv::Vec3d(0, 0, plane_z_));
      viz_.setWidgetPose("InputImagePlane", pose * plane_local);
    } catch (...) {
      // Widget doesn't exist yet, ignore.
    }
  }

  void updateCommon(const cv::Mat &input_img, const cv::Mat &bev_img,
                    const cv::Mat &undistorted_img) override {

    // 1. Input Image Plane (Undistorted)
    if (!undistorted_img.empty() && plane_w_ > 0) {
      try {
        // WImage3D: image, size, center, normal, up_vector
        cv::viz::WImage3D img_widget(
            undistorted_img, cv::Size2d(plane_w_, plane_h_),
            cv::Vec3d(0, 0, 0), // Center relative to widget pose
            cv::Vec3d(0, 0, 1), // Normal
            cv::Vec3d(0, 1, 0)  // Up (Corrected for upside-down)
        );
        viz_.showWidget("InputImagePlane", img_widget);

        // Re-apply pose because recreating widget might reset it (or not, but
        // safer to set) We need current camera pose. Since we don't store it
        // explicitly besides widget, we assume setCameraPose is called
        // frequently or we query it.
        cv::Affine3d cam_pose = viz_.getWidgetPose("CameraFrustum");
        cv::Affine3d plane_local =
            cv::Affine3d().translate(cv::Vec3d(0, 0, plane_z_));
        viz_.setWidgetPose("InputImagePlane", cam_pose * plane_local);

      } catch (...) {
      }
    }

    // 2. BEV Plane on Ground
    if (!bev_img.empty() && bev_width_m_ > 0) {
      try {
        cv::Vec3d center(bev_width_m_ / 2.0, 0, 0.05);
        cv::Vec3d normal(0, 0, 1);
        cv::Vec3d up(-1, 0, 0);

        if (!T_robot_bev_.empty()) {
          // Derive pose from T_robot_bev
          // Center is translation part
          center = cv::Vec3d(T_robot_bev_.at<double>(0, 3),
                             T_robot_bev_.at<double>(1, 3),
                             T_robot_bev_.at<double>(2, 3));

          // BEV Plane Z-bias to avoid z-fighting if at 0
          center[2] += 0.05;

          // Normal is Z_virt axis (Col 2)
          normal = cv::Vec3d(T_robot_bev_.at<double>(0, 2),
                             T_robot_bev_.at<double>(1, 2),
                             T_robot_bev_.at<double>(2, 2));

          // Up Vector for WImage3D correponds to Image Top direction (v
          // decreasing). Image Y axis is Y_virt (Col 1). Steps down. So Up
          // ("Top") is -Y_virt.
          cv::Vec3d y_virt(T_robot_bev_.at<double>(0, 1),
                           T_robot_bev_.at<double>(1, 1),
                           T_robot_bev_.at<double>(2, 1));

          // User requested invert: So flip the logic.
          // Try Up = y_virt.
          up = y_virt;
        }

        cv::viz::WImage3D bev_widget(bev_img,
                                     cv::Size2d(bev_width_m_, bev_height_m_),
                                     center, normal, up);
        viz_.showWidget("BEVPlane", bev_widget);
      } catch (...) {
      }
    }

    viz_.spinOnce(1, true);
  }

  void setPointPairs(const std::vector<cv::Point2f> &src,
                     const std::vector<cv::Point2f> &dst) override {
    // Placeholder
  }

  void spinOnce() override {
    // Handled in updateCommon to sync with viz loop
  }

private:
  cv::viz::Viz3d viz_;
  cv::Mat K_;
  cv::Size img_size_;
  double plane_w_ = 0, plane_h_ = 0, plane_z_ = 1.0;

  float bev_width_m_ = 0, bev_height_m_ = 0;
  cv::Mat T_robot_bev_;
};

} // namespace bev
#endif
