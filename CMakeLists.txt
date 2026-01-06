cmake_minimum_required(VERSION 3.14)
project(bev_opencv LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Options ---
option(BEV_ENABLE_CV_VIZ "Enable OpenCV Viz module for visualization" ON)
option(BEV_ENABLE_ROS_VIZ "Enable ROS 2 Rviz support" OFF)

# --- Dependencies ---
find_package(OpenCV 4.0 REQUIRED)
message(STATUS "OpenCV found: ${OpenCV_VERSION}")

if(BEV_ENABLE_CV_VIZ)
    if(NOT "${OpenCV_VIZ_FOUND}" AND NOT "viz" IN_LIST OpenCV_MODULES)
        find_package(OpenCV 4.0 REQUIRED COMPONENTS viz)
    endif()
    add_compile_definitions(BEV_ENABLE_CV_VIZ)
endif()

if(BEV_ENABLE_ROS_VIZ)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    add_compile_definitions(BEV_ENABLE_ROS_VIZ)
endif()

# --- Sources ---
file(GLOB_RECURSE BEV_SOURCES src/*.cpp)
file(GLOB_RECURSE BEV_HEADERS include/**/*.hpp)

# --- Executable ---
add_executable(bev_app app/main.cpp ${BEV_SOURCES} ${BEV_HEADERS})

target_include_directories(bev_app PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(bev_app PRIVATE ${OpenCV_LIBS})

# Test Suite
add_executable(test_kitti_projection tests/test_kitti_projection.cpp src/camera_model.cpp src/ipm_geometric.cpp src/ipm_homography.cpp)
target_link_libraries(test_kitti_projection ${OpenCV_LIBS})
target_include_directories(test_kitti_projection PRIVATE include)

if(BEV_ENABLE_ROS_VIZ)
    target_link_libraries(bev_app PRIVATE
        rclcpp::rclcpp
        sensor_msgs::sensor_msgs
        visualization_msgs::visualization_msgs
        nav_msgs::nav_msgs
        geometry_msgs::geometry_msgs
        cv_bridge::cv_bridge
    )
endif()
