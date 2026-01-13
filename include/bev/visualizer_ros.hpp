#pragma once
#include "visualizer_interface.hpp"

#ifdef BEV_ENABLE_ROS_VIZ
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace bev {

class VisualizerROS : public VisualizerInterface {
public:
  VisualizerROS() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    node_ = rclcpp::Node::make_shared("bev_visualizer");
    image_pub_ =
        node_->create_publisher<sensor_msgs::msg::Image>("bev/image", 10);
    undistorted_pub_ =
        node_->create_publisher<sensor_msgs::msg::Image>("bev/undistorted", 10);
  }

  void init() override {}
  void setCameraIntrinsics(const cv::Mat &K,
                           const cv::Size &img_size) override {}
  void setBEVGeometry(float width_m, float height_m, float res_m_px,
                      const cv::Mat &T_robot_bev) override {}

  void updateCommon(const cv::Mat &input_img, const cv::Mat &bev_img,
                    const cv::Mat &undistorted_img) override {
    std_msgs::msg::Header header;
    header.frame_id = "map"; // Fixed frame
    header.stamp = node_->now();

    auto publish =
        [&](rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub,
            const cv::Mat &img) {
          if (!img.empty() && pub) {
            std::string encoding = (img.type() == CV_8UC3) ? "bgr8" : "mono8";
            try {
              sensor_msgs::msg::Image::SharedPtr msg =
                  cv_bridge::CvImage(header, encoding, img).toImageMsg();
              pub->publish(*msg);
            } catch (...) {
            }
          }
        };

    publish(image_pub_, bev_img);
    publish(undistorted_pub_, undistorted_img);
  }

  void setCameraPose(const cv::Mat &T_robot_cam) override {}

  void setPointPairs(const std::vector<cv::Point2f> &src,
                     const std::vector<cv::Point2f> &dst) override {}

  void spinOnce() override { rclcpp::spin_some(node_); }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr undistorted_pub_;
};

} // namespace bev
#endif
