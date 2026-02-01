# ArUco 6D Pose Estimation Workspace

This workspace provides an optimized node for detecting ArUco markers, estimating their 6D pose, and visualizing them with 3D bounding boxes and axes using a RealSense D435i camera.
Website for Aruco Tag generate: https://chev.me/arucogen/ (Dictionary: Original ArUco)

## 🚀 Quick Start Guide

To run the system, you need to open **3 separate terminals**. Follow these steps exactly:

```bash
colcon build --packages-select opencv_tools
```
### Terminal 1: Camera Driver
```bash
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

### Terminal 2: ArUco Processor (Combined Launch)
To start both the camera and the processor with a single command:
```bash
source install/setup.bash
ros2 run opencv_tools aruco_processor
```

### Terminal 3: Visualization (RQT)
```bash
# IMPORTANT: Run this in a clean terminal WITHOUT the virtual environment active
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```
*In RQT, select the topic:* `/aruco/image_annotated`

*In Bash for Obj pose, select the topic:* `/output`
```bash
ros2 topic echo --once /output
```
---

## 🛠️ Environment Details
- **ROS Version**: ROS 2 Jazzy
- **Python Version**: 3.12 (inside `aruco_env_new`)
- **Main Node**: `aruco_processor` (in `opencv_tools` package)

## 📌 Features
- **3D Visualization**: Shows 3D bounding boxes (Green base, Red pillars, Blue top) and coordinate axes.
- **Auto-Calibration**: Automatically reads camera intrinsics from the RealSense topics.
- **Robust Pose Estimation**: Uses `cv2.solvePnP` for compatibility with modern OpenCV (4.7+).
- **TF Broadcasting**: Publishes transforms for each detected marker as `aruco_marker_<ID>`.

## ⚙️ Parameters
You can customize the node behavior:
```bash
ros2 run opencv_tools aruco_processor --ros-args -p marker_size:=0.07 -p dictionary:=DICT_4X4_50
```
- `marker_size`: Length of the marker side in meters (default: `0.05`).
- `dictionary`: ArUco dictionary to use (default: `DICT_ARUCO_ORIGINAL`).
