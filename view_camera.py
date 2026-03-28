#!/usr/bin/env python3
"""Simple camera viewer — subscribes to /camera/image_raw and shows it with cv2.imshow."""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    def __init__(self, topic: str):
        super().__init__("camera_viewer")
        self.bridge = CvBridge()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(Image, topic, self._cb, qos)
        self.get_logger().info(f"Subscribing to {topic} — press 'q' to quit")

    def _cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/camera/image_raw"
    rclpy.init()
    node = CameraViewer(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
