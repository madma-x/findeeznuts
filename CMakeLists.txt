cmake_minimum_required(VERSION 3.16)
project(findeeznuts)

# ── Compiler standard ──────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── CPU performance flags ──────────────────────────────────────────────────────
add_compile_options(
  -O3
  -flto=auto
  -fomit-frame-pointer
  -funroll-loops
  -ffast-math
  # Cache-friendly & vectorisation hints
  -fno-plt
  # Reduce binary size for deployment
  -fvisibility=hidden
)

# ARM NEON/SIMD for Raspberry Pi 4 (AArch64) – skip on other architectures
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
  add_compile_options(
    -march=armv8-a+crc+simd
    -mtune=cortex-a72
  )
endif()

add_link_options(
  -flto=auto
  -Wl,-O2
  -Wl,--as-needed
  # Strip symbols in Release builds
  $<$<CONFIG:Release>:-s>
)

# ── ROS2 dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(aruco_interfaces REQUIRED)

# ── OpenCV (must include opencv_aruco / contrib) ───────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d objdetect)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found. Install libopencv-contrib-dev.")
endif()
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# ── Boost (lock-free queues) ───────────────────────────────────────────────────
find_package(Boost REQUIRED)

# ── Main node ─────────────────────────────────────────────────────────────────
add_executable(aruco_picker_node
  src/aruco_picker_node.cpp
)

add_executable(camera_capture_node
  src/camera_capture_node.cpp
)

target_link_libraries(aruco_picker_node
  ${OpenCV_LIBS}
  pthread
)

target_link_libraries(camera_capture_node
  ${OpenCV_LIBS}
)

ament_target_dependencies(aruco_picker_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  aruco_interfaces
  cv_bridge
  image_transport
)

ament_target_dependencies(camera_capture_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(aruco_picker_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_include_directories(camera_capture_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  aruco_picker_node
  camera_capture_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/aruco_pose_debug_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
