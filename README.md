# ArUco 6D Pose Estimation Workspace

This workspace provides an optimized node for detecting ArUco markers, estimating their 6D pose, and visualizing them with 3D bounding boxes and axes using a RealSense D435i camera.

## 🚀 Quick Start Guide

To run the system, you need to open **3 separate terminals**. Follow these steps exactly:

### Terminal 1: Camera Driver
```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

### Terminal 2: ArUco Processor (Combined Launch)
To start both the camera and the processor with a single command:
```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/Documents/aruco_ws/install/setup.bash
ros2 launch opencv_tools aruco_multi.launch.py
```

### 📌 Multi-Tag Configuration
If you have markers of different sizes, you can specify them using the `marker_sizes` parameter (JSON format):
```bash
ros2 launch opencv_tools aruco_multi.launch.py marker_sizes:='{"0": 0.05, "1": 0.12}'
```
- Marker ID `0` is 5cm.
- Marker ID `1` is 12cm.
- Any other detected markers will use the default `marker_size` (0.05m).

### Terminal 3: Visualization (RQT)
```bash
# IMPORTANT: Run this in a clean terminal WITHOUT the virtual environment active
conda deactivate
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view
```
*In RQT, select the topic:* `/aruco/image_annotated`

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





header:
  stamp:
    sec: 1769680209
    nanosec: 116428711
  frame_id: camera_color_optical_frame
poses:
- position:
    x: 0.1953449306048083
    y: 0.0027433058560438054
    z: 0.5379345201479184
  orientation:
    x: -0.26246015330163197
    y: -0.1569153618687383
    z: -0.6660395415862801
    w: 0.6803554704579252
---