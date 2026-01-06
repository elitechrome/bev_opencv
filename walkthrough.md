# BEV OpenCV Project - Walkthrough

I have implemented the Bird's Eye View (BEV) generation system using OpenCV C++ with support for two methods: **Homography** and **Geometric (Orthographic)**.

## Key Features
*   **Geometric IPM**: Rigorous ray-casting from a virtual Orthographic camera to the ground plane, verifying intersection and re-projecting to the input camera.
*   **Fisheye Support**: Handles `distortion_model: fisheye` using `cv::fisheye` module for undistortion and projection.
*   **Homography IPM**: Standard 4-point perspective transform.
*   **Visualization**: Modular architecture supporting **OpenCV Viz** (VTK) and **ROS 2 Rviz** (via macros).
*   **Configuration**: Full YAML support for intrinsics and geometric parameters.

## Build Instructions

```bash
mkdir build
cd build
cmake ..
make
```

### Build Options
*   `-DBEV_ENABLE_CV_VIZ=ON` (Default): Builds with `cv::viz`.
*   `-DBEV_ENABLE_ROS_VIZ=ON`: Builds with ROS 2 support (requires explicitly sourcing ROS 2 env).

## Running the Application

### 1. Geometric Mode
The default configuration uses the Geometric method.

```bash
# Run from project root
./build/bev_app config/bev_config_sample.yaml
```

**Expected Output:**
```
Starting BEV Application with config: config/bev_config_sample.yaml
[CameraModel] Loaded intrinsics from camera_intrinsics.yaml (Model: Standard)
[IPMGeometric] Precomputing maps...
[IPMGeometric] Maps generated.
BEV App initialized in geometric mode.
Using checkerboard pattern.
```

### 2. Fisheye Support
To use a fisheye camera, update `camera_intrinsics.yaml`:
```yaml
distortion_model: "fisheye"
camera_matrix: ...
dist_coeffs: ...
```
The application will automatically detect this and switch to `cv::fisheye` functions.
*   A window "BEV Visualization" should appear (if HEADLESS is not set).
*   It displays the input image (Checkerboard if no image provided) and the generated BEV.

### 3. Homography Mode
Edit `config/bev_config_sample.yaml` and set:
```yaml
mode: "homography"
```
Re-run the application to verify the homography warping.

## File Structure
*   `src/ipm_geometric.cpp`: Core logic for ray interaction and map generation.
*   `src/camera_model.cpp`: Handles intrinsics and undistortion.
*   `include/bev/visualizer_cv.hpp` & `..._ros.hpp`: Header-only implementations for visualization.
