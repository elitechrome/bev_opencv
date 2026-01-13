cmake_minimum_required(VERSION 3.14)
project(bev_opencv LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV 4.0 REQUIRED COMPONENTS core imgproc calib3d highgui)
find_package(yaml-cpp REQUIRED)

# --- Library ---
file(GLOB_RECURSE BEV_SOURCES src/*.cpp)
# Exclude main files from library if they were accidentally picked up (though they are in app/)
list(FILTER BEV_SOURCES EXCLUDE REGEX ".*app/.*")

add_library(bev_lib ${BEV_SOURCES})
target_include_directories(bev_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(bev_lib ${OpenCV_LIBS} yaml-cpp)

# --- Executables ---

# 1. Standalone App (Optional, but good to keep)
add_executable(bev_app app/main.cpp)
target_link_libraries(bev_app bev_lib ${OpenCV_LIBS})

# 2. ROS 2 Node
add_executable(bev_node app/ros2_app.cpp)
target_include_directories(bev_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(bev_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  nav_msgs
  geometry_msgs
  visualization_msgs
)
target_link_libraries(bev_node bev_lib ${OpenCV_LIBS} yaml-cpp)

# 3. Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_executable(test_kitti_projection tests/test_kitti_projection.cpp)
  target_link_libraries(test_kitti_projection bev_lib ${OpenCV_LIBS})
  target_include_directories(test_kitti_projection PRIVATE include)

  add_executable(test_bev_synthetic tests/test_bev_synthetic.cpp)
  target_link_libraries(test_bev_synthetic bev_lib ${OpenCV_LIBS})
  target_include_directories(test_bev_synthetic PRIVATE include)
endif()

# --- Install Rules ---

# Install Targets (Executables and Libraries)
install(TARGETS bev_lib bev_app bev_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# --- Export ---
ament_export_include_directories(include)
ament_export_libraries(bev_lib)
ament_export_dependencies(rclcpp sensor_msgs cv_bridge OpenCV yaml-cpp)

ament_package()
