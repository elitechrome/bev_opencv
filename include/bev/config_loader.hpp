#pragma once

#include "bev/types.hpp"
#include <string>

namespace bev {

class ConfigLoader {
public:
  static SystemConfig load(const std::string &yaml_path);
};

} // namespace bev
