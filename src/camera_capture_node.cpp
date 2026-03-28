#include <chrono>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraCaptureNode : public rclcpp::Node {
public:
  CameraCaptureNode() : rclcpp::Node("camera_capture") {
    declare_parameter("backend", std::string("gstreamer"));
    declare_parameter("device_id", 0);
    declare_parameter("camera_topic", std::string("/camera/image_raw"));
    declare_parameter("frame_id", std::string("camera_optical_frame"));
    declare_parameter("width", 640);
    declare_parameter("height", 480);
    declare_parameter("fps", 30.0);
    declare_parameter(
      "gst_pipeline",
      std::string(
        "libcamerasrc ! "
        "video/x-raw,width=640,height=480,framerate=30/1 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"));

    backend_ = get_parameter("backend").as_string();
    device_id_ = get_parameter("device_id").as_int();
    camera_topic_ = get_parameter("camera_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    fps_ = get_parameter("fps").as_double();
    gst_pipeline_ = get_parameter("gst_pipeline").as_string();

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::Image>(camera_topic_, qos);

    const auto period_ms = std::max(1, static_cast<int>(1000.0 / std::max(1.0, fps_)));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&CameraCaptureNode::capture_once, this));

    RCLCPP_INFO(
      get_logger(),
      "camera_capture started | backend=%s topic=%s %dx%d@%.1f",
      backend_.c_str(), camera_topic_.c_str(), width_, height_, fps_);
  }

private:
  void ensure_camera_open() {
    if (capture_.isOpened()) {
      return;
    }

    if (backend_ == "gstreamer") {
      capture_.open(gst_pipeline_, cv::CAP_GSTREAMER);
    } else {
      capture_.open(device_id_, cv::CAP_V4L2);
    }

    if (!capture_.isOpened()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to open camera backend '%s'. Retrying...", backend_.c_str());
      return;
    }

    if (backend_ != "gstreamer") {
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
      capture_.set(cv::CAP_PROP_FPS, fps_);
    }

    if (backend_ == "gstreamer") {
      RCLCPP_INFO(get_logger(), "Opened camera via gstreamer pipeline");
    } else {
      RCLCPP_INFO(get_logger(), "Opened /dev/video%d", device_id_);
    }
    read_fail_count_ = 0;
  }

  void capture_once() {
    ensure_camera_open();
    if (!capture_.isOpened()) {
      return;
    }

    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      ++read_fail_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Camera read failed (%d consecutive).", read_fail_count_);

      // Recreate pipeline/device only after repeated failures.
      if (read_fail_count_ >= 10) {
        capture_.release();
        read_fail_count_ = 0;
      }
      return;
    }
    read_fail_count_ = 0;

    cv_bridge::CvImage out;
    out.header.stamp = now();
    out.header.frame_id = frame_id_;
    out.encoding = "bgr8";
    out.image = frame;
    publisher_->publish(*out.toImageMsg());
  }

  int device_id_{0};
  int width_{640};
  int height_{480};
  double fps_{30.0};
  int read_fail_count_{0};
  std::string backend_;
  std::string camera_topic_;
  std::string frame_id_;
  std::string gst_pipeline_;

  cv::VideoCapture capture_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraCaptureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
