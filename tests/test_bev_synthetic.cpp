#include "bev/config_loader.hpp"
#include "bev/geometric_processor.hpp"
#include "bev/lut_optimized_processor.hpp"
#include "bev/sequential_processor.hpp"
#include <cmath>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <vector>

// Logic to check parallelism
// Returns std_dev of lane width (distance between centroids of left and right
// lane)
// Logic to check parallelism
// Returns std_dev of lane width (distance between centroids of left and right
// lane) and saves a debug image.
double measureParallelism(const cv::Mat &bev_image,
                          const std::string &debug_name) {
  // 1. Threshold to get binary mask of lines
  cv::Mat mask;
  cv::Mat debug_img;

  if (bev_image.channels() == 3) {
    debug_img = bev_image.clone();
    cv::Mat gray;
    cv::cvtColor(bev_image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 100, 255, cv::THRESH_BINARY);
  } else {
    cv::cvtColor(bev_image, debug_img, cv::COLOR_GRAY2BGR);
    cv::threshold(bev_image, mask, 100, 255, cv::THRESH_BINARY);
  }

  // 2. Find row-wise centroids
  std::vector<double> widths;
  int mid_x = mask.cols / 2;

  for (int y = 0; y < mask.rows; y += 10) { // Step 10 for visibility
    std::vector<int> left_indices;
    std::vector<int> right_indices;

    const uchar *row_ptr = mask.ptr<uchar>(y);
    for (int x = 0; x < mask.cols; ++x) {
      if (row_ptr[x] > 0) {
        if (x < mid_x)
          left_indices.push_back(x);
        else
          right_indices.push_back(x);
      }
    }

    if (!left_indices.empty() && !right_indices.empty()) {
      double avg_left =
          std::accumulate(left_indices.begin(), left_indices.end(), 0.0) /
          left_indices.size();
      double avg_right =
          std::accumulate(right_indices.begin(), right_indices.end(), 0.0) /
          right_indices.size();
      widths.push_back(avg_right - avg_left);

      // Draw points
      cv::circle(debug_img, cv::Point((int)avg_left, y), 2,
                 cv::Scalar(0, 0, 255), -1);
      cv::circle(debug_img, cv::Point((int)avg_right, y), 2,
                 cv::Scalar(255, 0, 0), -1);
      cv::line(debug_img, cv::Point((int)avg_left, y),
               cv::Point((int)avg_right, y), cv::Scalar(0, 255, 0), 1);
    }
  }

  if (widths.size() < 10) {
    std::cerr << "Not enough line segments found in BEV!" << std::endl;
    return 1000.0; // Fail
  }

  // Calculate Mean and StdDev
  double sum = std::accumulate(widths.begin(), widths.end(), 0.0);
  double mean = sum / widths.size();

  double sq_sum =
      std::inner_product(widths.begin(), widths.end(), widths.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / widths.size() - mean * mean);

  std::cout << "  Mean Lane Width: " << mean << " px" << std::endl;
  std::cout << "  Width Std Dev: " << stdev << " px" << std::endl;

  // Draw stats
  std::string stats = "Mean: " + std::to_string((int)mean) +
                      " px, StdDev: " + std::to_string(stdev).substr(0, 5);
  cv::putText(debug_img, stats, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX,
              1.0, cv::Scalar(0, 255, 255), 2);

  cv::imwrite(debug_name, debug_img);
  std::cout << "  Saved debug visualization to " << debug_name << std::endl;

  return stdev;
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  std::cout << "[Test] Synthetic Data Verification" << std::endl;

  // 1. Load Config
  std::string config_path = "config/kitti_config.yaml";
  bev::SystemConfig config;
  try {
    config = bev::ConfigLoader::load(config_path);
  } catch (std::exception &e) {
    std::cerr << "Failed to load config: " << e.what() << std::endl;
    return 1;
  }

  // 2. Load Synthetic Image
  std::string image_path = "data/kitti_sample.png";
  cv::Mat input_img = cv::imread(image_path);
  if (input_img.empty()) {
    std::cerr << "Failed to load image: " << image_path << std::endl;
    return 1;
  }
  std::cout << "Loaded " << image_path << " " << input_img.size() << std::endl;

  // 3. Processors to Test
  std::vector<std::string> names = {"Sequential", "LutOptimized", "Geometric"};
  std::vector<std::unique_ptr<bev::IVisionPipeline>> processors;
  processors.push_back(std::make_unique<bev::SequentialProcessor>(config));
  processors.push_back(std::make_unique<bev::LutOptimizedProcessor>(config));
  processors.push_back(std::make_unique<bev::GeometricProcessor>(config));

  bool all_passed = true;

  for (size_t i = 0; i < processors.size(); ++i) {
    std::cout << "\nTesting " << names[i] << "..." << std::endl;
    cv::Mat output = processors[i]->process(input_img);

    // Save for visual inspection
    std::string out_name = "output_" + names[i] + ".png";
    cv::imwrite(out_name, output);
    std::cout << "  Saved output to " << out_name << std::endl;

    // A. Check Visibility (Should have some white pixels)
    int white_pixels =
        cv::countNonZero(output.channels() == 3 ? [](const cv::Mat &src) {
          cv::Mat g;
          cv::cvtColor(src, g, cv::COLOR_BGR2GRAY);
          return g;
        }(output)
                                                : output);

    std::cout << "  Non-zero pixels: " << white_pixels << std::endl;
    if (white_pixels < 100) {
      std::cerr << "  FAIL: Image mostly black/empty." << std::endl;
      all_passed = false;
      continue;
    }

    // B. Check Parallelism
    std::string debug_viz_name = "visualized_" + names[i] + ".png";
    double std_dev = measureParallelism(output, debug_viz_name);
    if (std_dev > 10.0) { // Tolerance in pixels
      std::cerr << "  FAIL: Lines are not parallel (StdDev " << std_dev
                << " > 10.0)" << std::endl;
      all_passed = false;
    } else {
      std::cout << "  PASS: Parallelism check (StdDev " << std_dev << ")"
                << std::endl;
    }
  }

  if (all_passed) {
    std::cout << "\nAll Processors Passed." << std::endl;
    return 0;
  } else {
    std::cerr << "\nSome tests failed." << std::endl;
    return 1;
  }
}
