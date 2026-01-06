# BEV (Bird's Eye View) System Specification

## 1. Overview
This system generates a Bird's Eye View (BEV) image from a single camera input using Inverse Perspective Mapping (IPM). It supports two methods for IPM generation: a point-correspondence method and a rigorous geometric pose-based method. Use C++ and OpenCV.

## 2. Input Data
1.  **Source Image**: Raw 2D image from the camera.
2.  **Intrinsic Calibration**: OpenCV YAML file containing:
    *   Camera Matrix ($K$)
    *   Distortion Coefficients ($D$)
3.  **Coordinate Systems**:
    *   **Robot Frame**:
        *   Origin: Robot center (or specific reference point)
        *   X-axis: Front
        *   Y-axis: Left
        *   Z-axis: Up
    *   **Camera Pose**: Transformation from Robot to Camera ($T_{robot\_cam}$).
    *   **Virtual BEV Camera Pose**: Transformation from Robot to Virtual Camera ($T_{robot\_bev}$).

## 3. Functional Requirements

### 3.1. Pre-processing
*   **Undistortion**: 
    *   Load camera intrinsics from YAML.
    *   Generate undistortion maps.
    *   **Debug Output**: Display the undistorted image for verification.

### 3.2. IPM Generation Method 1: Point Correspondence (Homography)
*   User provides **4 distinct points** on the *Input Image* (undistorted).
*   User provides **4 corresponding points** on the *Target BEV Image* (Ground plane coordinates converted to pixels).
*   Compute Homography matrix $H$ using `cv::findHomography`.
*   Warp image using $H$.

### 3.3. IPM Generation Method 2: Geometric Pose (Orthographic)
*   **Parameters**:
    *   Input Camera Pose ($T_{rc}$ given in Robot frame).
    *   Virtual Camera Pose ($T_{rv}$ given in Robot frame).
    *   Virtual Camera Intrinsics:
        *   **orthographic** projection (no FOV).
        *   Target Resolution (Width, Height).
        *   Scale: `px/mm` (Pixels per millimeter on ground).
*   **Coordinate System Handling**:
    *   **Crucial**: The implementation **must** explicitly handle the rotation from the Robot Frame (X-front, Y-left, Z-up) to the Camera Optical Frame (X-right, Y-down, Z-forward).
    *   $T_{optical} = T_{robot\_cam} \times R_{robot\_to\_optical}$
*   **Process**:
    1.  Define the ground plane in the Robot frame (typically $Z=0$).
    2.  For each pixel $(u, v)$ in the Target BEV Image (Orthographic grid):
        *   Calculate world coordinate $P_{ground}$ based on scale (`px/mm`) and image center.
        *   Project $P_{ground}$ from World/Robot space back into the Input Camera's pixel coordinates $(u', v')$ using the intrinsic matrix and $T_{optical}$.
    3.  Generate Map_X and Map_Y for `cv::remap`.

### 3.4. Optimization
*   Use `cv::remap` for the actual image warping in the main loop.
*   Pre-calculate the look-up maps (`map1`, `map2`) during initialization or when parameters change.

## 4. Visualization & GUI
The visualization module should be modular and buildable based on preprocessor macros.

### Option A: OpenCV Native (`CV_VIZ` macro)
*   **Library**: `cv::viz` (VTK-based).
*   **Functionality**:
    *   Independent 3D window.
    *   Visualizes Robot Frame, Camera Frustums (Input & Virtual).
    *   Renders the texturized Ground Plane.
    *   Visualizes point correspondences (lines connecting source center to target ground points).

### Option B: ROS 2 / Rviz (`ROS_VIZ` macro)
*   **Library**: ROS 2 implementation (publishers).
*   **Functionality**:
    *   Publish `sensor_msgs::msg::Image` for Input and BEV images.
    *   Publish `visualization_msgs::msg::Marker` or `MarkerArray` for:
        *   Camera Frustums (Line lists).
        *   Coordinate Axes.
    *   Publish `nav_msgs::msg::Odometry` or `geometry_msgs::msg::PoseStamped` for camera poses.
*   **Rviz Configuration**: Provide a `.rviz` file to visualize these topics easily.

### Build Configuration
*   Use CMake options to toggle macros `CV_VIZ` and `ROS_VIZ`.
*   Code sections relevant to typical VIZ logic must be wrapped in `#ifdef` blocks to prevent compile errors when dependencies are missing.
*   **2D Windows**:
    *   Input Image (Raw/Undistorted).
    *   Output BEV Image.

## 5. Implementation Constraints
*   **Language**: C++ (C++14 or C++17 recommended).
*   **Library**: OpenCV 4.x.
*   **Build System**: CMake, with options to enable/disable `CV_VIZ` and `ROS_VIZ` support.

