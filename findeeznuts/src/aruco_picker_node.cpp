/**
 * aruco_picker_node.cpp
 *
 * CPU-optimised ROS2 node for ArUco detection, pose tracking and Y-axis
 * clustering on Raspberry Pi 4 (AArch64 / ARM NEON).
 *
 * Pipeline (three dedicated threads pinned to CPU cores):
 *   Thread 0 (core 0) – ROS2 spin / image capture
 *   Thread 1 (core 1) – ArUco detection
 *   Thread 2 (core 2) – Alignment + publish
 *
 * Inter-thread communication uses boost::lockfree::spsc_queue (wait-free,
 * cache-line padded) to avoid mutex contention in hot paths.
 *
 * Memory layout:
 *   - Pre-allocated ring buffers (no heap allocation in hot paths)
 *   - Cache-line aligned structs (64 bytes on ARM)
 *   - Object-pool for detected tag results
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <vector>

// Boost lock-free SPSC queue (header-only, no dynamic allocation)
#include <boost/lockfree/spsc_queue.hpp>

// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

// ROS2
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// Generated messages
#include "aruco_picker/msg/detected_tag.hpp"
#include "aruco_picker/msg/detected_tag_array.hpp"

// ── Constants ─────────────────────────────────────────────────────────────────

static constexpr int kQueueCapacity   = 4;   // ring-buffer depth per stage
static constexpr int kMaxTags         = 64;  // maximum ArUco markers per frame
static constexpr float kSmoothAlpha   = 0.7f; // EMA: higher = faster response
static constexpr float kLostDecay     = 0.85f; // confidence decay when tag lost
static constexpr float kConfirmAlpha  = 0.15f; // confidence rise per detection
static constexpr int kLostFrameLimit  = 10;   // frames without detection → drop

// ── Cache-line aligned data structures ───────────────────────────────────────

struct alignas(64) Vec3f {
  float x{0}, y{0}, z{0};
  Vec3f() = default;
  Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
  Vec3f operator+(const Vec3f& o) const { return {x+o.x, y+o.y, z+o.z}; }
  Vec3f operator-(const Vec3f& o) const { return {x-o.x, y-o.y, z-o.z}; }
  Vec3f operator*(float s) const { return {x*s, y*s, z*s}; }
};

struct alignas(64) Quatf {
  float w{1}, x{0}, y{0}, z{0};
};

/// Single detected / tracked ArUco tag (fits in two cache lines)
struct alignas(64) TagState {
  uint32_t id{0};
  Vec3f    pos;             // smoothed translation (metres)
  Quatf    rot;             // smoothed rotation (quaternion)
  float    confidence{0.0f};
  int      lost_frames{0};
  bool     active{false};
};

/// Lightweight frame token passed through queues (no image copy)
struct FrameToken {
  sensor_msgs::msg::Image::ConstSharedPtr msg;  // shared-ownership, zero-copy
  uint64_t seq{0};
};

/// Detection result – passed from detection thread to cluster thread
struct alignas(64) DetectionResult {
  struct RawTag {
    uint32_t id;
    Vec3f    tvec;  // translation from solvePnP
    Quatf    rvec_q; // rotation as quaternion
  };
  std::array<RawTag, kMaxTags> tags{};
  int       count{0};
  uint64_t  seq{0};
  rclcpp::Time stamp;
};

// ── Utility: pin calling thread to a CPU core ─────────────────────────────────

static void pin_thread_to_core(int core_id)
{
#if defined(__linux__)
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
#else
  (void)core_id;
#endif
}

// ── Utility: Rodrigues vector → unit quaternion ──────────────────────────────

static Quatf rvec_to_quat(const cv::Vec3d& rvec)
{
  const double angle = std::sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);
  if (angle < 1e-9) return {1.0f, 0.0f, 0.0f, 0.0f};
  const double s = std::sin(angle * 0.5) / angle;
  return {
    static_cast<float>(std::cos(angle * 0.5)),
    static_cast<float>(rvec[0] * s),
    static_cast<float>(rvec[1] * s),
    static_cast<float>(rvec[2] * s)
  };
}

// ── Utility: quaternion SLERP (used in EMA smoothing) ───────────────────────

static Quatf quat_slerp(const Quatf& a, const Quatf& b, float t)
{
  float dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
  Quatf b2 = b;
  if (dot < 0.0f) {
    // take the shorter arc
    b2 = {-b.w, -b.x, -b.y, -b.z};
    dot = -dot;
  }
  if (dot > 0.9995f) {
    // nearly identical – linear interpolation is fine
    Quatf r{a.w + t*(b2.w-a.w), a.x + t*(b2.x-a.x),
            a.y + t*(b2.y-a.y), a.z + t*(b2.z-a.z)};
    float n = std::sqrt(r.w*r.w + r.x*r.x + r.y*r.y + r.z*r.z);
    return {r.w/n, r.x/n, r.y/n, r.z/n};
  }
  const float theta0  = std::acos(dot);
  const float theta   = theta0 * t;
  const float sin0    = std::sin(theta0);
  const float sinT    = std::sin(theta);
  const float s0 = std::cos(theta) - dot * sinT / sin0;
  const float s1 = sinT / sin0;
  return {
    s0*a.w + s1*b2.w,
    s0*a.x + s1*b2.x,
    s0*a.y + s1*b2.y,
    s0*a.z + s1*b2.z
  };
}

// ═════════════════════════════════════════════════════════════════════════════
//  ArucoPicker node
// ═════════════════════════════════════════════════════════════════════════════

class ArucoPickerNode : public rclcpp::Node
{
public:
  explicit ArucoPickerNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : rclcpp::Node("aruco_picker", opts)
  {
    // ── Declare + read parameters ──────────────────────────────────────────
    declare_parameter("camera_topic",       "/camera/image_raw");
    declare_parameter("marker_size",        0.05);   // metres
    declare_parameter("aruco_dict",         10);     // DICT_6X6_250
    declare_parameter("theoretical_positions",
      std::vector<double>{-0.025, -0.025, 0.5,
                           0.025, -0.025, 0.5,
                           0.025,  0.025, 0.5,
                          -0.025,  0.025, 0.5});
    declare_parameter("theoretical_ids",
      std::vector<int64_t>{0, 1, 2, 3});
    declare_parameter("target_fps",         30.0);
    declare_parameter("frame_skip_on_lag",  true);
    declare_parameter("camera_matrix",
      std::vector<double>{600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0});
    declare_parameter("dist_coeffs",
      std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter("core_capture",   0);
    declare_parameter("core_detect",    1);
    declare_parameter("core_publish",   2);

    camera_topic_     = get_parameter("camera_topic").as_string();
    marker_size_      = static_cast<float>(get_parameter("marker_size").as_double());
    frame_skip_       = get_parameter("frame_skip_on_lag").as_bool();
    core_capture_     = get_parameter("core_capture").as_int();
    core_detect_      = get_parameter("core_detect").as_int();
    core_publish_     = get_parameter("core_publish").as_int();

    // ── Theoretical layout ─────────────────────────────────────────────────
    auto tp = get_parameter("theoretical_positions").as_double_array();
    auto ti = get_parameter("theoretical_ids").as_integer_array();
    if (tp.size() >= 12 && ti.size() >= 4) {
      for (int i = 0; i < 4; ++i) {
        theoretical_pos_[i] = {
          static_cast<float>(tp[i*3+0]),
          static_cast<float>(tp[i*3+1]),
          static_cast<float>(tp[i*3+2])
        };
        theoretical_ids_[i] = static_cast<uint32_t>(ti[i]);
      }
      has_theoretical_ = true;
    } else {
      RCLCPP_WARN(get_logger(),
        "theoretical_positions/ids not fully set – alignment will use detected centroid only");
    }

    // ── Camera calibration matrices ────────────────────────────────────────
    auto cm = get_parameter("camera_matrix").as_double_array();
    auto dc = get_parameter("dist_coeffs").as_double_array();
    camera_matrix_ = (cv::Mat_<double>(3,3) <<
      cm[0], cm[1], cm[2],
      cm[3], cm[4], cm[5],
      cm[6], cm[7], cm[8]);
    dist_coeffs_ = cv::Mat(dc, true).reshape(1, 1);

    // ── ArUco detector (pre-compiled, reused across frames) ───────────────
    int dict_id = get_parameter("aruco_dict").as_int();
    auto dict   = cv::aruco::getPredefinedDictionary(dict_id);
    cv::aruco::DetectorParameters det_params;
    // Speed optimisations: relax corner refinement for throughput
    det_params.cornerRefinementMethod       = cv::aruco::CORNER_REFINE_NONE;
    det_params.adaptiveThreshWinSizeMin     = 3;
    det_params.adaptiveThreshWinSizeMax     = 23;
    det_params.adaptiveThreshWinSizeStep    = 10;
    det_params.minMarkerPerimeterRate       = 0.03;
    det_params.maxMarkerPerimeterRate       = 4.0;
    det_params.polygonalApproxAccuracyRate  = 0.03;
    detector_ = std::make_unique<cv::aruco::ArucoDetector>(dict, det_params);

    // ── Publisher ──────────────────────────────────────────────────────────
    // Keep-last(1) best-effort – only the latest detection matters
    rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
    pub_qos.best_effort().durability_volatile();
    publisher_ = create_publisher<aruco_picker::msg::DetectedTagArray>(
      "~/detected_tags", pub_qos);

    // ── Stats timer (1 Hz) ─────────────────────────────────────────────────
    stats_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() { print_stats(); });

    // ── Launch pipeline threads ────────────────────────────────────────────
    running_.store(true, std::memory_order_release);
    detect_thread_  = std::thread(&ArucoPickerNode::detect_loop,  this);
    publish_thread_ = std::thread(&ArucoPickerNode::publish_loop, this);

    // ── Camera subscriber (best-effort, no retries) ────────────────────────
    // Subscribed last so threads are ready before frames arrive.
    rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
    sub_qos.best_effort().durability_volatile();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, sub_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        on_image(std::move(msg));
      });

    RCLCPP_INFO(get_logger(),
      "aruco_picker started | topic=%s | cores=[%d,%d,%d]",
      camera_topic_.c_str(), core_capture_, core_detect_, core_publish_);
  }

  ~ArucoPickerNode() override
  {
    running_.store(false, std::memory_order_release);
    if (detect_thread_.joinable())  detect_thread_.join();
    if (publish_thread_.joinable()) publish_thread_.join();
  }

private:
  // ── Stage 0 – image callback (runs in the ROS2 spin thread, core 0) ────────

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    // Pin the ROS2 callback thread to the capture core (lazy – first call)
    static std::once_flag once;
    std::call_once(once, [this]() { pin_thread_to_core(core_capture_); });

    frame_count_in_.fetch_add(1, std::memory_order_relaxed);

    if (frame_skip_) {
      // Drop frame if the detection queue is full (processing is lagging)
      FrameToken token{msg, seq_.fetch_add(1, std::memory_order_relaxed)};
      if (!capture_queue_.push(token)) {
        frames_dropped_.fetch_add(1, std::memory_order_relaxed);
      }
    } else {
      FrameToken token{msg, seq_.fetch_add(1, std::memory_order_relaxed)};
      // Blocking: spin until there is space (not recommended, prefer skip)
      while (!capture_queue_.push(token)) {
        std::this_thread::yield();
      }
    }
  }

  // ── Stage 1 – detection thread (core 1) ────────────────────────────────────

  void detect_loop()
  {
    pin_thread_to_core(core_detect_);

    // Scratch buffers – re-used across frames (no per-frame allocation)
    std::vector<int>                    ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<cv::Vec3d>              rvecs, tvecs;

    ids.reserve(kMaxTags);
    corners.reserve(kMaxTags);
    rvecs.reserve(kMaxTags);
    tvecs.reserve(kMaxTags);

    while (running_.load(std::memory_order_acquire)) {
      FrameToken token;
      if (!capture_queue_.pop(token)) {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
        continue;
      }

      // Convert ROS image to OpenCV Mat without copying pixel data
      cv_bridge::CvImageConstPtr cv_img;
      try {
        cv_img = cv_bridge::toCvShare(token.msg, "bgr8");
      } catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "cv_bridge error: %s", e.what());
        continue;
      }
      const cv::Mat& frame = cv_img->image;
      if (frame.empty()) continue;

      // Grey-scale conversion (in-place, NEON-accelerated by OpenCV)
      cv::Mat grey;
      cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);

      // ArUco detection
      ids.clear(); corners.clear(); rejected.clear();
      detector_->detectMarkers(grey, corners, ids, rejected);

      DetectionResult result;
      result.seq   = token.seq;
      result.stamp = rclcpp::Time(token.msg->header.stamp);
      result.count = 0;

      if (!ids.empty()) {
        rvecs.resize(ids.size());
        tvecs.resize(ids.size());

        // Pose estimation for all detected markers
        for (size_t i = 0; i < ids.size() && result.count < kMaxTags; ++i) {
          cv::solvePnP(
            build_object_points(marker_size_),
            corners[i],
            camera_matrix_, dist_coeffs_,
            rvecs[i], tvecs[i],
            false, cv::SOLVEPNP_IPPE_SQUARE);

          auto& t = result.tags[result.count];
          t.id     = static_cast<uint32_t>(ids[i]);
          t.tvec   = {
            static_cast<float>(tvecs[i][0]),
            static_cast<float>(tvecs[i][1]),
            static_cast<float>(tvecs[i][2])
          };
          t.rvec_q = rvec_to_quat(rvecs[i]);
          ++result.count;
        }
      }

      detect_count_.fetch_add(1, std::memory_order_relaxed);

      // Push to publish queue (drop if full – detection is faster than publish)
      if (!detect_queue_.push(result)) {
        results_dropped_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  // ── Stage 2 – alignment + publish thread (core 2) ─────────────────────────

  void publish_loop()
  {
    pin_thread_to_core(core_publish_);

    while (running_.load(std::memory_order_acquire)) {
      DetectionResult result;
      if (!detect_queue_.pop(result)) {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
        continue;
      }

      // 1. Update tracker states
      update_tracker(result);

      // 2. Collect active tags
      std::vector<const TagState*> active;
      active.reserve(kMaxTags);
      for (const auto& ts : tracker_) {
        if (ts.active && ts.confidence > 0.1f) active.push_back(&ts);
      }

      // 3. Compute alignment point from theoretical layout + detected tags
      auto align = compute_alignment(active);

      // 4. Build and publish message
      auto out_msg = std::make_unique<aruco_picker::msg::DetectedTagArray>();
      out_msg->header.stamp    = result.stamp;
      out_msg->header.frame_id = "camera_optical_frame";
      out_msg->tags.reserve(active.size());

      for (size_t ti = 0; ti < active.size(); ++ti) {
        const auto* ts = active[ti];
        aruco_picker::msg::DetectedTag tag_msg;
        tag_msg.tag_id = ts->id;

        tag_msg.tag_pose.position.x    = static_cast<double>(ts->pos.x);
        tag_msg.tag_pose.position.y    = static_cast<double>(ts->pos.y);
        tag_msg.tag_pose.position.z    = static_cast<double>(ts->pos.z);
        tag_msg.tag_pose.orientation.w = static_cast<double>(ts->rot.w);
        tag_msg.tag_pose.orientation.x = static_cast<double>(ts->rot.x);
        tag_msg.tag_pose.orientation.y = static_cast<double>(ts->rot.y);
        tag_msg.tag_pose.orientation.z = static_cast<double>(ts->rot.z);
        tag_msg.confidence             = ts->confidence;

        // Theoretical slot this tag was matched to (-1 = unmatched)
        tag_msg.slot_id = align.tag_to_slot[ti];

        // Shared alignment point for the whole object group
        tag_msg.alignment_pose.position.x    = static_cast<double>(align.point.x);
        tag_msg.alignment_pose.position.y    = static_cast<double>(align.point.y);
        tag_msg.alignment_pose.position.z    = static_cast<double>(align.point.z);
        tag_msg.alignment_pose.orientation.w = 1.0;

        out_msg->tags.push_back(std::move(tag_msg));
      }

      publisher_->publish(std::move(out_msg));
      publish_count_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  // ── Tracker update (EMA smoothing + confidence scoring) ──────────────────

  void update_tracker(const DetectionResult& result)
  {
    // Mark all active states as "not seen this frame"
    for (auto& ts : tracker_) {
      if (ts.active) ++ts.lost_frames;
    }

    for (int i = 0; i < result.count; ++i) {
      const auto& raw = result.tags[i];
      TagState* ts = find_or_create(raw.id);
      if (!ts) continue;

      ts->lost_frames = 0;

      if (!ts->active) {
        // First detection: initialise directly
        ts->pos        = raw.tvec;
        ts->rot        = raw.rvec_q;
        ts->confidence = kConfirmAlpha;
        ts->active     = true;
      } else {
        // EMA smoothing
        ts->pos = ts->pos * (1.0f - kSmoothAlpha) + raw.tvec * kSmoothAlpha;
        ts->rot = quat_slerp(ts->rot, raw.rvec_q, kSmoothAlpha);
        ts->confidence = std::min(1.0f,
          ts->confidence * (1.0f - kConfirmAlpha) + kConfirmAlpha);
      }
    }

    // Decay / deactivate lost tags
    for (auto& ts : tracker_) {
      if (ts.active && ts.lost_frames > 0) {
        ts.confidence *= kLostDecay;
        if (ts.lost_frames > kLostFrameLimit || ts.confidence < 0.05f) {
          ts.active     = false;
          ts.confidence = 0.0f;
        }
      }
    }
  }

  TagState* find_or_create(uint32_t id)
  {
    // O(N) scan – N ≤ kMaxTags, stays in L1 cache due to alignment
    for (auto& ts : tracker_) {
      if (ts.active && ts.id == id) return &ts;
    }
    // Reuse first inactive slot
    for (auto& ts : tracker_) {
      if (!ts.active) {
        ts      = TagState{};
        ts.id   = id;
        return &ts;
      }
    }
    return nullptr; // pool exhausted
  }

  // ── Theoretical-layout alignment ─────────────────────────────────────────

  /// Result returned by compute_alignment()
  struct AlignmentResult {
    Vec3f            point{};          // alignment point in camera frame
    int              matched_count{0}; // number of detected tags matched to a slot
    std::vector<int> tag_to_slot;      // per active-tag slot index (0-3, or -1)
  };

  /// Compute the alignment point the robot should move toward.
  ///
  /// Algorithm:
  ///   1. Match each detected tag to one of the 4 theoretical slots:
  ///      - first by marker ID (O(4) scan per tag)
  ///      - then by 3-D proximity for any remaining unmatched tags
  ///   2. For every matched slot compute: offset = detected_pos − theoretical_pos
  ///   3. alignment_point = theoretical_centre + mean(offsets)
  ///
  /// When fewer than 4 tags are visible the undetected slots contribute no
  /// offset, so the alignment point stays close to the theoretical centre and
  /// only drifts by the average error of what is actually seen.
  AlignmentResult compute_alignment(const std::vector<const TagState*>& active)
  {
    AlignmentResult ar;
    ar.tag_to_slot.assign(active.size(), -1);

    // Theoretical centre (fixed, independent of detections)
    Vec3f centre;
    for (const auto& p : theoretical_pos_) centre = centre + p;
    centre = centre * 0.25f;

    if (!has_theoretical_ || active.empty()) {
      ar.point = centre;
      return ar;
    }

    // One detected tag per slot at most
    std::array<int, 4> slot_to_tag{-1, -1, -1, -1};  // slot → active[] index
    std::vector<bool>  tag_used(active.size(), false);

    // Pass 1: ID-based matching
    for (size_t ti = 0; ti < active.size(); ++ti) {
      for (int si = 0; si < 4; ++si) {
        if (slot_to_tag[si] < 0 && active[ti]->id == theoretical_ids_[si]) {
          slot_to_tag[si]   = static_cast<int>(ti);
          ar.tag_to_slot[ti] = si;
          tag_used[ti]       = true;
          break;
        }
      }
    }

    // Pass 2: proximity matching for tags not assigned by ID
    for (size_t ti = 0; ti < active.size(); ++ti) {
      if (tag_used[ti]) continue;
      float best_dist = std::numeric_limits<float>::max();
      int   best_slot = -1;
      for (int si = 0; si < 4; ++si) {
        if (slot_to_tag[si] >= 0) continue;  // slot already taken
        const Vec3f d = active[ti]->pos - theoretical_pos_[si];
        const float dist = d.x*d.x + d.y*d.y + d.z*d.z;
        if (dist < best_dist) { best_dist = dist; best_slot = si; }
      }
      if (best_slot >= 0) {
        slot_to_tag[best_slot]  = static_cast<int>(ti);
        ar.tag_to_slot[ti]      = best_slot;
        tag_used[ti]            = true;
      }
    }

    // Average offset from all matched slots
    Vec3f avg_offset;
    for (int si = 0; si < 4; ++si) {
      if (slot_to_tag[si] < 0) continue;
      avg_offset = avg_offset + (active[slot_to_tag[si]]->pos - theoretical_pos_[si]);
      ++ar.matched_count;
    }
    if (ar.matched_count > 0) {
      avg_offset = avg_offset * (1.0f / static_cast<float>(ar.matched_count));
    }

    ar.point = centre + avg_offset;
    return ar;
  }

  // ── Object-point template for solvePnP ───────────────────────────────────

  static std::vector<cv::Point3f> build_object_points(float size)
  {
    const float h = size * 0.5f;
    return {
      {-h,  h, 0.0f},
      { h,  h, 0.0f},
      { h, -h, 0.0f},
      {-h, -h, 0.0f}
    };
  }

  // ── Diagnostics ───────────────────────────────────────────────────────────

  void print_stats()
  {
    const uint64_t in  = frame_count_in_.exchange(0, std::memory_order_relaxed);
    const uint64_t det = detect_count_.exchange(0,  std::memory_order_relaxed);
    const uint64_t pub = publish_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t fdr = frames_dropped_.exchange(0, std::memory_order_relaxed);
    const uint64_t rdr = results_dropped_.exchange(0, std::memory_order_relaxed);

    RCLCPP_INFO(get_logger(),
      "[FPS] in=%lu det=%lu pub=%lu | dropped frames=%lu results=%lu",
      in, det, pub, fdr, rdr);
  }

  // ── Member variables ──────────────────────────────────────────────────────

  // Parameters
  std::string camera_topic_;
  float       marker_size_;
  bool        frame_skip_;
  int         core_capture_;
  int         core_detect_;
  int         core_publish_;

  // Theoretical object layout (4 marker positions + their IDs)
  std::array<Vec3f, 4>    theoretical_pos_{};
  std::array<uint32_t, 4> theoretical_ids_{};
  bool                    has_theoretical_{false};

  // Camera intrinsics
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // ArUco detector (thread-safe read after init)
  std::unique_ptr<cv::aruco::ArucoDetector> detector_;

  // Publisher
  rclcpp::Publisher<aruco_picker::msg::DetectedTagArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          image_sub_;
  rclcpp::TimerBase::SharedPtr                                       stats_timer_;

  // Lock-free single-producer single-consumer queues
  boost::lockfree::spsc_queue<FrameToken,
    boost::lockfree::capacity<kQueueCapacity>> capture_queue_;
  boost::lockfree::spsc_queue<DetectionResult,
    boost::lockfree::capacity<kQueueCapacity>> detect_queue_;

  // Tracker pool (pre-allocated, cache-line aligned)
  std::array<TagState, kMaxTags> tracker_{};

  // Pipeline threads
  std::thread       detect_thread_;
  std::thread       publish_thread_;
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> seq_{0};

  // Stats counters
  std::atomic<uint64_t> frame_count_in_{0};
  std::atomic<uint64_t> detect_count_{0};
  std::atomic<uint64_t> publish_count_{0};
  std::atomic<uint64_t> frames_dropped_{0};
  std::atomic<uint64_t> results_dropped_{0};
};

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true); // zero-copy within process

  auto node = std::make_shared<ArucoPickerNode>(opts);

  // SingleThreadedExecutor: minimal overhead, ROS2 callbacks on core 0
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
