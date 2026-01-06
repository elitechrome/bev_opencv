#include "bev/camera_model.hpp"
#include "bev/ipm_geometric.hpp"
#include "bev/ipm_homography.hpp"
#include "bev/visualizer_cv.hpp"
#include "bev/visualizer_ros.hpp"
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: ./bev_app <config_path>" << std::endl;
    return 1;
  }

  std::string config_path = argv[1];
  std::cout << "Starting BEV Application with config: " << config_path
            << std::endl;

  cv::FileStorage config(config_path, cv::FileStorage::READ);
  if (!config.isOpened()) {
    std::cerr << "Failed to open config: " << config_path << std::endl;
    return 1;
  }

  // 1. Setup Camera Model
  bev::CameraModel camera_model;
  std::string intrinsics_path;
  config["camera"]["intrinsics_path"] >> intrinsics_path;

  if (!camera_model.loadFromYaml(intrinsics_path)) {
    std::cerr << "Failed to load intrinsics: " << intrinsics_path << std::endl;
    return 1;
  }

  // 2. Setup IPM Strategy
  std::unique_ptr<bev::IPMStrategy> ipm;
  std::string mode;
  config["mode"] >> mode;

  cv::Size target_size(800, 800);
  bev::GeometricParams params;

  if (mode == "homography") {
    auto strategy = std::make_unique<bev::IPMHomography>();
    std::vector<cv::Point2f> src_pts, dst_pts;
    cv::FileNode node = config["homography"];
    if (node.empty()) {
      std::cerr << "Homography config missing" << std::endl;
      return 1;
    }

    auto readPoints = [](const cv::FileNode &n, std::vector<cv::Point2f> &pts) {
      for (auto &&e : n) {
        std::vector<float> vec;
        e >> vec;
        if (vec.size() == 2)
          pts.emplace_back(vec[0], vec[1]);
      }
    };

    readPoints(node["src_points"], src_pts);
    readPoints(node["dst_points"], dst_pts);

    int w = 500, h = 500;
    node["target_width"] >> w;
    node["target_height"] >> h;
    target_size = cv::Size(w, h);

    strategy->init(camera_model.getParams(), src_pts, dst_pts, target_size);
    ipm = std::move(strategy);

  } else if (mode == "geometric") {
    auto strategy = std::make_unique<bev::IPMGeometric>();
    cv::FileNode node = config["geometric"];
    if (node.empty()) {
      std::cerr << "Geometric config missing" << std::endl;
      return 1;
    }

    node["T_robot_cam"] >> params.T_robot_camera;
    node["T_robot_bev"] >> params.T_robot_bev;
    node["target_res_width"] >> params.target_res_width;
    node["target_res_height"] >> params.target_res_height;
    node["px_per_mm"] >> params.scale_px_per_mm;

    target_size =
        cv::Size((int)params.target_res_width, (int)params.target_res_height);

    strategy->init(camera_model.getParams(), params);
    ipm = std::move(strategy);
  }

  std::cout << "BEV App initialized in " << mode << " mode." << std::endl;

  // 3. Initialize Visualizer
  std::unique_ptr<bev::VisualizerInterface> visualizer;
#ifdef BEV_ENABLE_CV_VIZ
  visualizer = std::make_unique<bev::VisualizerCV>();
#elif defined(BEV_ENABLE_ROS_VIZ)
  visualizer = std::make_unique<bev::VisualizerROS>();
#else
  std::cerr << "No visualizer compiled in!" << std::endl;
  // return 1; // Or run headless
#endif
  if (visualizer) {
    visualizer->init();
    visualizer->setCameraIntrinsics(camera_model.getParams().K,
                                    camera_model.getParams().image_size);

    if (mode == "geometric") {
      float w_meters =
          params.target_res_width * (1.0f / params.scale_px_per_mm / 1000.0f);
      float h_meters =
          params.target_res_height * (1.0f / params.scale_px_per_mm / 1000.0f);
      float res_meters = 1.0f / params.scale_px_per_mm / 1000.0f;

      visualizer->setBEVGeometry(w_meters, h_meters, res_meters,
                                 params.T_robot_bev);
      visualizer->setCameraPose(params.T_robot_camera);
    }
  }

  // 4. Load Input
  std::string image_path;
  config["input_image"] >> image_path;
  cv::Mat input_img = cv::imread(image_path);

  if (input_img.empty()) {
    std::cout << "Using checkerboard pattern." << std::endl;
    input_img = cv::Mat::zeros(720, 1280, CV_8UC3);
    int sq = 100;
    for (int y = 0; y < 720; y += sq)
      for (int x = 0; x < 1280; x += sq)
        if ((x / sq + y / sq) % 2 == 0)
          cv::rectangle(input_img, cv::Rect(x, y, sq, sq),
                        cv::Scalar(255, 255, 255), -1);
  }

  // 5. Loop
  while (true) {
    cv::Mat bev_img;
    ipm->process(input_img, bev_img);

    cv::Mat undistorted_img;
    if (visualizer) {
      undistorted_img = camera_model.undistort(input_img);
      visualizer->updateCommon(input_img, bev_img, undistorted_img);
      visualizer->spinOnce();
    }

    if (!bev_img.empty()) {
      cv::imshow("BEV Output", bev_img);
    }

    if (cv::waitKey(30) == 27)
      break;
  }

  return 0;
}
