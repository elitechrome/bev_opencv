#include "bev/config_loader.hpp"
#include "bev/lut_optimized_processor.hpp"
#include "bev/sequential_processor.hpp"
#include "bev/vision_pipeline.hpp"
#include "bev/visualizer_cv.hpp"
#include "bev/visualizer_ros.hpp"

#include <iostream>
#include <memory>
#include <string>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: ./bev_app <config_path> [strategy: sequential|lut]"
              << std::endl;
    return 1;
  }

  std::string config_path = argv[1];
  std::string strategy_name =
      (argc > 2) ? argv[2] : "lut"; // Default to LUT as per SDD

  std::cout << "Starting BEV Application with config: " << config_path
            << std::endl;
  std::cout << "Strategy: " << strategy_name << std::endl;

  try {
    // 1. Load Configuration
    auto config = bev::ConfigLoader::load(config_path);

    // 2. Setup Pipeline
    std::unique_ptr<bev::IVisionPipeline> pipeline;
    if (strategy_name == "sequential") {
      pipeline = std::make_unique<bev::SequentialProcessor>(config);
    } else {
      pipeline = std::make_unique<bev::LutOptimizedProcessor>(config);
    }

    // 3. Initialize Visualizer (Optional / Debug)
    std::unique_ptr<bev::VisualizerInterface> visualizer;
#ifdef BEV_ENABLE_CV_VIZ
    visualizer = std::make_unique<bev::VisualizerCV>();
#elif defined(BEV_ENABLE_ROS_VIZ)
    visualizer = std::make_unique<bev::VisualizerROS>();
#endif

    if (visualizer) {
      visualizer->init();
      // Visualizer expects K and Image Size.
      // We assume image size 640x480 for now or derive from K principal point *
      // 2? Let's use hardcoded 640x480 since Input definition in SDD 1.2 is
      // 640x480.
      visualizer->setCameraIntrinsics(config.intrinsics.K, cv::Size(640, 480));

      // Geometry for Viz
      float w_meters = config.target_resolution.width * config.meter_per_px;
      float h_meters = config.target_resolution.height * config.meter_per_px;
      float res_meters = config.meter_per_px;

      // T_robot_bev corresponds to T_robot_virtual in our config
      visualizer->setBEVGeometry(w_meters, h_meters, res_meters,
                                 config.T_robot_virtual);
      visualizer->setCameraPose(config.T_robot_cam);
    }

    // 4. Input Setup (Image or Stream)
    cv::VideoCapture cap;
    cv::Mat input_img;
    bool use_stream = (config.input_source == "stream");

    if (use_stream) {
      std::cout << "Attempting to open camera ID: " << config.camera_id
                << std::endl;
      if (!cap.open(config.camera_id, cv::CAP_V4L2)) {
        std::cerr << "Failed to open camera ID " << config.camera_id
                  << ". Falling back to image/checkerboard." << std::endl;
        use_stream = false;
      } else {
        // Try to set resolution if we can, to match SDD
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
      }
    }

    if (!use_stream) {
      if (!config.input_image_path.empty()) {
        input_img = cv::imread(config.input_image_path);
      }

      // RESIZE FIX: Removed hardcoded resize to support dynamic resolution.
      if (!input_img.empty()) {
        std::cout << "Loaded input image size: " << input_img.size()
                  << std::endl;
      }

      if (input_img.empty()) {
        std::cout << "No valid input image found in config or failed to load. "
                     "Using checkerboard pattern."
                  << std::endl;
        input_img = cv::Mat::zeros(480, 640, CV_8UC3); // Standard size 640x480
        int sq = 50;
        for (int y = 0; y < 480; y += sq)
          for (int x = 0; x < 640; x += sq)
            if ((x / sq + y / sq) % 2 == 0)
              cv::rectangle(input_img, cv::Rect(x, y, sq, sq),
                            cv::Scalar(255, 255, 255), -1);
      }
    }

    // 5. Processing Loop
    std::cout << "Press ESC to exit." << std::endl;
    int frame_count = 0;
    while (true) {
      if (use_stream) {
        cap >> input_img;
        if (input_img.empty()) {
          std::cerr << "Stream ended or failed to capture frame." << std::endl;
          break;
        }
        // Enforce size on stream too if necessary
        if (input_img.size() != cv::Size(640, 480)) {
          // Warning: resolution mismatch might affect intrinsics if not
          // calibrated for this size. But we allow it now.
        }

        // DRAW LIVE INDICATOR to prove it flows
        cv::putText(input_img, "LIVE STREAM #" + std::to_string(frame_count++),
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 0, 255), 2);
      }

      // Run Pipeline
      cv::Mat bev_map = pipeline->process(
          input_img); // Returns Mask (CV_8UC1 probably, or Float?)

      pipeline->printStats();

      // Visualization
      if (visualizer) {
        // Visualizer expects BGR usually? Or handle mono.
        // Also expects "Undistorted Image".
        // We can compute undistorted image using cv::undistort for viz
        // purposes.
        cv::Mat undistorted;
        cv::undistort(input_img, undistorted, config.intrinsics.K,
                      config.intrinsics.D);

        visualizer->updateCommon(input_img, bev_map, undistorted);
        visualizer->spinOnce();
      }

      if (!input_img.empty()) {
        cv::imshow("Raw Camera Preview", input_img);
      }

      if (!bev_map.empty()) {
        cv::imshow("BEV Output (Mask)", bev_map);
      }

      if (cv::waitKey(1) == 27) // Smaller wait for stream
        break;
    }

  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
