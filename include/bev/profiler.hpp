#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

namespace bev {

class ScopedTimer {
public:
  ScopedTimer(std::string_view name,
              std::vector<std::pair<std::string, double>> &registry)
      : name_(name), registry_(registry),
        start_(std::chrono::high_resolution_clock::now()) {}

  ~ScopedTimer() {
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start_;
    registry_.emplace_back(name_, elapsed.count());
  }

private:
  std::string name_;
  std::vector<std::pair<std::string, double>> &registry_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

} // namespace bev
