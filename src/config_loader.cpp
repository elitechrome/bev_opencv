#include "bev/config_loader.hpp"
#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace bev {

namespace {
// Helper to decode OpenCV matrix from YAML
cv::Mat parseMatrix(const YAML::Node &node) {
  if (!node.IsMap()) {
    throw std::runtime_error("Matrix node is not a map");
  }

  int rows = node["rows"].as<int>();
  int cols = node["cols"].as<int>();
  std::string dt = node["dt"].as<std::string>();

  std::vector<double> data = node["data"].as<std::vector<double>>();

  if (data.size() != static_cast<size_t>(rows * cols)) {
    throw std::runtime_error("Matrix data size mismatch");
  }

  cv::Mat mat(rows, cols, CV_64F);
  for (int i = 0; i < rows * cols; ++i) {
    mat.at<double>(i) = data[i];
  }
  return mat.clone();
}
} // namespace

SystemConfig ConfigLoader::load(const std::string &yaml_path) {
  SystemConfig config;
  try {
    YAML::Node yaml = YAML::LoadFile(yaml_path);

    if (!yaml["Camera"])
      throw std::runtime_error("Missing 'Camera' section");
    if (!yaml["Transforms"])
      throw std::runtime_error("Missing 'Transforms' section");

    auto camera_node = yaml["Camera"];
    config.intrinsics.K = parseMatrix(camera_node["K"]);
    config.intrinsics.D = parseMatrix(camera_node["D"]);
    config.intrinsics.distortion_model = camera_node["Model"].as<std::string>();

    auto transforms_node = yaml["Transforms"];
    config.T_robot_cam = parseMatrix(transforms_node["T_robot_cam"]);
    config.T_robot_virtual = parseMatrix(transforms_node["T_robot_virtual"]);

    // Local Input Configuration
    if (yaml["Input"]) {
      auto input_node = yaml["Input"];
      if (input_node["source"])
        config.input_source = input_node["source"].as<std::string>();
      if (input_node["camera_id"])
        config.camera_id = input_node["camera_id"].as<int>();
      if (input_node["image_path"])
        config.input_image_path = input_node["image_path"].as<std::string>();
    } else {
      // Backward compatibility for root-level keys
      if (yaml["input_image"])
        config.input_image_path = yaml["input_image"].as<std::string>();
      if (yaml["input_source"])
        config.input_source = yaml["input_source"].as<std::string>();
      if (yaml["camera_id"])
        config.camera_id = yaml["camera_id"].as<int>();
    }

    // Target Spec Configuration
    if (yaml["TargetSpec"]) {
      auto spec_node = yaml["TargetSpec"];
      if (spec_node["meter_per_px"])
        config.meter_per_px = spec_node["meter_per_px"].as<float>();
      if (spec_node["width"])
        config.target_resolution.width = spec_node["width"].as<int>();
      if (spec_node["height"])
        config.target_resolution.height = spec_node["height"].as<int>();
    } else {
      // Defaults
      config.meter_per_px = 0.02f;
      config.target_resolution = cv::Size(600, 600);
    }

  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to load config from " + yaml_path + ": " +
                             e.what());
  }

  return config;
}

} // namespace bev
