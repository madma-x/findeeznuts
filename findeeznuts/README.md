# aruco_picker

CPU-optimised ROS2 C++ package for real-time ArUco marker detection, pose
tracking, and Y-axis clustering on **Raspberry Pi 4** (Debian Trixie /
AArch64).  Written for the **CDR2026** competition.

---

## Features

| Feature | Details |
|---|---|
| Detection | OpenCV `ArucoDetector` – `DICT_6X6_250` by default |
| Pose estimation | `solvePnP` with `IPPE_SQUARE` solver |
| Tracking | Exponential Moving Average (α = 0.7) + confidence scoring |
| Clustering | Y-axis proximity (configurable threshold, default 5 cm) |
| Threading | 3-stage pipeline pinned to CPU cores 0/1/2 |
| Queues | `boost::lockfree::spsc_queue` (wait-free, no mutex) |
| SIMD | ARM NEON via `-march=armv8-a+crc+simd` + OpenCV NEON paths |
| Optimisation | `-O3 -flto` + stripped symbols for deployment |

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `~/detected_tags` | `aruco_picker/DetectedTagArray` | All tracked tags (QoS: best-effort, keep-last 1) |

### Message format

```
aruco_picker/DetectedTagArray
  std_msgs/Header header
  aruco_picker/DetectedTag[] tags
    uint32 tag_id
    geometry_msgs/Pose tag_pose       # smoothed individual pose
    int32  cluster_id                 # -1 = not clustered
    geometry_msgs/Pose cluster_pose   # centroid of cluster (if clustered)
    float32 confidence                # [0.0 – 1.0]
```

---

## Requirements

```bash
# Debian Trixie (Bookworm should also work)
sudo apt install \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport \
  libopencv-contrib-dev \
  libboost-dev
```

> **Note:** `libopencv-contrib-dev` provides `opencv_aruco` /
> `opencv_objdetect` with the new `cv::aruco::ArucoDetector` API (OpenCV 4.7+).

---

## Build

The repository is named `findeeznuts` and the ROS2 package inside it is
`aruco_picker` (this directory).  Copy or symlink the package directory into
your workspace `src/`:

```bash
cd ~/ros2_ws

# Option A – symlink (recommended for development)
# The target of the symlink must be the package directory (the one that
# contains package.xml), and the link name must be the ROS2 package name.
ln -s /path/to/findeeznuts/findeeznuts src/aruco_picker

# Option B – copy
cp -r /path/to/findeeznuts/findeeznuts src/aruco_picker

colcon build \
  --packages-select aruco_picker \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-march=armv8-a+crc+simd -O3 -flto=auto"

source install/setup.bash
```

---

## Camera calibration

Replace the default `camera_matrix` / `dist_coeffs` in
`config/aruco_picker.yaml` with your actual calibration:

```bash
# Using the ROS2 camera_calibration tool
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/camera/image_raw camera:=/camera
```

---

## Run

```bash
# Default camera topic (/camera/image_raw)
ros2 launch aruco_picker aruco_picker.launch.py

# Custom camera topic
ros2 launch aruco_picker aruco_picker.launch.py \
  camera_topic:=/raspicam_node/image
```

Check detections:

```bash
ros2 topic echo /aruco_picker/detected_tags
ros2 topic hz  /aruco_picker/detected_tags
```

---

## Performance tuning

### Image resolution

Lower resolution → higher FPS.  Set the camera driver to publish at
320 × 240 for maximum throughput, 640 × 480 for better detection range.

### CPU affinity

The node pins its three threads to cores 0 / 1 / 2 by default.
Change via YAML:

```yaml
core_capture: 0
core_detect:  1
core_publish: 2
```

Leave core 3 free for the OS and other ROS2 nodes.

### Frame skipping

`frame_skip_on_lag: true` (default) drops incoming frames when the detection
queue is full, preventing unbounded latency.  Set to `false` only if you need
every frame processed (e.g., offline bag replay).

### Compiler flags (manual build)

```bash
colcon build --packages-select aruco_picker \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-O3 -flto=auto -march=armv8-a+crc+simd \
                       -mtune=cortex-a72 -ffast-math"
```

### Benchmarking

```bash
# Monitor FPS (node prints stats every second)
ros2 topic echo /rosout | grep aruco_picker

# CPU usage per core
mpstat -P ALL 1

# End-to-end latency
ros2 topic delay /aruco_picker/detected_tags
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `error: 'ArucoDetector' is not a member of 'cv::aruco'` | Install `libopencv-contrib-dev` (OpenCV 4.7+) |
| No detections | Check `aruco_dict` matches the printed markers |
| Poor pose accuracy | Calibrate camera – default intrinsics are approximate |
| High latency | Enable `frame_skip_on_lag: true` or reduce resolution |
| 0 FPS on `/detected_tags` | Verify camera topic name with `ros2 topic list` |
