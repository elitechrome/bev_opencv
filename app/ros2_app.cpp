#include <cv_bridge/cv_bridge.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "bev/config_loader.hpp"
#include "bev/geometric_processor.hpp"
#include "bev/lut_optimized_processor.hpp"
#include "bev/sequential_processor.hpp"

using std::placeholders::_1;

class BevNode : public rclcpp::Node {
public:
  BevNode() : Node("bev_opencv_node") {
    // Parameters
    this->declare_parameter("config_file", "");
    this->declare_parameter("use_camera_info", true); // Default to using topic
    this->declare_parameter("strategy", "lut");       // lut or sequential

    config_file_ = this->get_parameter("config_file").as_string();
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    std::string strategy = this->get_parameter("strategy").as_string();

    if (config_file_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'config_file' is required.");
      // In a real app we might exit or shutdown, but for now we let it run (and
      // fail later) or wait.
    } else {
      loadConfig(config_file_);
    }

    // Subscriptions
    // Use image_transport later if needed, but standard sub is fine for raw
    // images
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/kitti/camera/color/left/image_raw", 10,
        std::bind(&BevNode::imageCallback, this, _1));

    if (use_camera_info_) {
      info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          "/kitti/camera/color/left/camera_info", 10,
          std::bind(&BevNode::infoCallback, this, _1));
    } else {
      // Initialize pipeline immediately if we trust YAML intrinsics
      initializePipeline(strategy);
    }

    // Publisher
    bev_pub_ = this->create_publisher<sensor_msgs::msg::Image>("bev/image", 10);

    RCLCPP_INFO(this->get_logger(),
                "BEV Node Initialized. Strategy: %s, Use CameraInfo: %s",
                strategy.c_str(), use_camera_info_ ? "true" : "false");
  }

private:
  void loadConfig(const std::string &path) {
    try {
      config_ = bev::ConfigLoader::load(path);
      config_loaded_ = true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
      config_loaded_ = false;
    }
  }

  void initializePipeline(const std::string &strategy) {
    if (!config_loaded_)
      return;

    if (strategy == "sequential") {
      pipeline_ = std::make_unique<bev::SequentialProcessor>(config_);
    } else if (strategy == "geometric") {
      pipeline_ = std::make_unique<bev::GeometricProcessor>(config_);
    } else {
      pipeline_ = std::make_unique<bev::LutOptimizedProcessor>(config_);
    }
    pipeline_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "Pipeline initialized.");
  }

  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (intrinsics_received_)
      return; // Only need it once usually? Or should we update if it changes?
    // For now, load once.

    if (!config_loaded_) {
      // We still need geometry from YAML, so we must wait for config_loaded
      // If config is not loaded yet (e.g. empty path?), we can't proceed.
      if (config_file_.empty())
        return;
    }

    // Update intrinsics from CameraInfo
    // K is 3x3 array
    cv::Mat K = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
    cv::Mat D =
        cv::Mat(1, msg->d.size(), CV_64F, (void *)msg->d.data()).clone();

    config_.intrinsics.K = K;
    config_.intrinsics.D = D;
    config_.intrinsics.distortion_model = msg->distortion_model;

    RCLCPP_INFO(this->get_logger(), "Intrinsics updated from CameraInfo.");

    std::string strategy = this->get_parameter("strategy").as_string();
    initializePipeline(strategy);
    intrinsics_received_ = true;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!pipeline_ready_) {
      // If we are not waiting for camera info, we might have failed config
      // load. If waiting for camera info, assume it hasn't arrived.
      static int throttle = 0;
      if (throttle++ % 100 == 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "Pipeline not ready. Config loaded: %d, Intrinsics Needed: %d",
            config_loaded_, use_camera_info_);
      }
      return;
    }

    try {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      cv::Mat input_img = cv_ptr->image;
      // if (input_img.size() != cv::Size(640, 480)) {
      //   cv::resize(input_img, input_img, cv::Size(640, 480));
      // }

      cv::Mat result = pipeline_->process(input_img);

      if (!result.empty()) {
        sensor_msgs::msg::Image::SharedPtr out_msg =
            cv_bridge::CvImage(msg->header, "mono8", result).toImageMsg();
        bev_pub_->publish(*out_msg);
      }
      cv::waitKey(1);

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bev_pub_;

  std::string config_file_;
  bool use_camera_info_;
  bool config_loaded_ = false;
  bool pipeline_ready_ = false;
  bool intrinsics_received_ = false;

  bev::SystemConfig config_;
  std::unique_ptr<bev::IVisionPipeline> pipeline_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BevNode>());
  rclcpp::shutdown();
  return 0;
}
