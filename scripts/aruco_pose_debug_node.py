#!/usr/bin/env python3
"""
aruco_pose_debug_node.py

Subscribes to a camera image and a DetectedTagArray, then publishes an
annotated debug image showing for each active tag:
  - Projected tag centre (crosshair + circle)
  - Tag ID, XYZ pose and distance in cm
  - Confidence bar (red → yellow → green)

Published topic: ~/debug_image  (sensor_msgs/Image, BGR8)

Parameters:
  camera_topic   – raw camera image topic  (default: /camera/image_raw)
  tags_topic     – DetectedTagArray topic  (default: /aruco_picker/detected_tags)
  show_window    – open local cv2 window   (default: false, use on desktop only)
  camera_matrix  – 9-element flat array matching aruco_picker.yaml
"""

import math

import cv2
import numpy as np
import rclpy
from aruco_interfaces.msg import DetectedTagArray
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# ── Helpers ───────────────────────────────────────────────────────────────────

FONT       = cv2.FONT_HERSHEY_SIMPLEX
FONT_SMALL = 0.45
FONT_MED   = 0.55
LINE       = cv2.LINE_AA
WHITE      = (255, 255, 255)
DARK       = (30,  30,  30)


def _conf_color(conf: float):
    """BGR colour: red (0) → yellow (0.5) → green (1)."""
    conf = max(0.0, min(1.0, conf))
    if conf < 0.5:
        r = 255
        g = int(255 * conf * 2)
    else:
        r = int(255 * (1.0 - conf) * 2)
        g = 255
    return (0, g, r)


def _shadow_text(img, text, org, font, scale, color, thickness=1):
    """Draw text with a dark outline for readability on any background."""
    cv2.putText(img, text, org, font, scale, DARK,  thickness + 1, LINE)
    cv2.putText(img, text, org, font, scale, color, thickness,     LINE)


# ═════════════════════════════════════════════════════════════════════════════
class ArucoPosgDebugNode(Node):

    def __init__(self):
        super().__init__("aruco_pose_debug")

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("tags_topic",   "/findeeznuts/detected_tags")
        self.declare_parameter("show_window",  False)
        self.declare_parameter("camera_matrix",
            [600.0, 0.0, 320.0,
               0.0, 600.0, 240.0,
               0.0,   0.0,   1.0])

        cam_topic  = self.get_parameter("camera_topic").value
        tags_topic = self.get_parameter("tags_topic").value
        self._show = self.get_parameter("show_window").value
        cm = self.get_parameter("camera_matrix").value

        self._fx = float(cm[0])
        self._fy = float(cm[4])
        self._cx = float(cm[2])
        self._cy = float(cm[5])

        self._bridge      = CvBridge()
        self._latest_tags = None   # type: DetectedTagArray | None

        # ── Subscriptions ─────────────────────────────────────────────────────
        self._tag_sub = self.create_subscription(
            DetectedTagArray, tags_topic, self._tags_cb, be_qos)

        self._img_sub = self.create_subscription(
            Image, cam_topic, self._image_cb, be_qos)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Image, "~/debug_image", be_qos)

        self.get_logger().info(
            f"aruco_pose_debug ready | cam={cam_topic} | tags={tags_topic} "
            f"| publish → ~/debug_image")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _tags_cb(self, msg: DetectedTagArray):
        self._latest_tags = msg

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge error: {exc}", throttle_duration_sec=2.0)
            return

        out = self._draw_overlays(frame)

        out_msg          = self._bridge.cv2_to_imgmsg(out, encoding="bgr8")
        out_msg.header   = msg.header
        self._pub.publish(out_msg)

        if self._show:
            cv2.imshow("ArUco Pose Debug", out)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                cv2.destroyAllWindows()
                rclpy.shutdown()

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _draw_overlays(self, frame: np.ndarray) -> np.ndarray:
        out  = frame.copy()
        h, w = out.shape[:2]

        tags = self._latest_tags
        if tags is None or not tags.tags:
            _shadow_text(out, "No tags detected", (10, 30), FONT, FONT_MED,
                         (0, 60, 255), thickness=1)
            return out

        for tag in tags.tags:
            p    = tag.tag_pose.position
            x, y, z = float(p.x), float(p.y), float(p.z)
            conf = float(tag.confidence)
            color = _conf_color(conf)

            # Skip tags behind the camera
            if z <= 0.001:
                continue

            # ── Project tag centre to pixel (pinhole, ignore distortion) ──────
            u = int(self._fx * (x / z) + self._cx)
            v = int(self._fy * (y / z) + self._cy)

            # Clamp to image bounds for drawing (can still be off-screen)
            u_clamp = max(0, min(w - 1, u))
            v_clamp = max(0, min(h - 1, v))

            dist_cm = math.sqrt(x*x + y*y + z*z) * 100.0

            # ── Crosshair + circle ─────────────────────────────────────────────
            R = 16
            cv2.circle(out, (u, v), R, color, 2, LINE)
            cv2.line(out, (u - R, v), (u + R, v), color, 1, LINE)
            cv2.line(out, (u, v - R), (u, v + R), color, 1, LINE)

            # ── Text lines above the crosshair ────────────────────────────────
            lines = [
                f"ID:{tag.tag_id}   {dist_cm:.1f} cm",
                f"x:{x*100:+.1f}  y:{y*100:+.1f}  z:{z*100:.1f} cm",
            ]
            line_h = 18
            text_base_y = v - R - 8
            for i, line in enumerate(reversed(lines)):
                ty = text_base_y - i * line_h
                _shadow_text(out, line, (u - 55, ty), FONT, FONT_SMALL, color)

            # ── Confidence bar below the crosshair ────────────────────────────
            bar_w  = 64
            bar_h  = 8
            bx     = u - bar_w // 2
            by     = v + R + 6
            # Background
            cv2.rectangle(out, (bx, by), (bx + bar_w, by + bar_h), (50, 50, 50), -1)
            # Fill
            filled = max(1, int(bar_w * conf))
            cv2.rectangle(out, (bx, by), (bx + filled, by + bar_h), color, -1)
            # Border
            cv2.rectangle(out, (bx, by), (bx + bar_w, by + bar_h), (150, 150, 150), 1)
            # Value label
            _shadow_text(out, f"{conf:.2f}",
                         (bx + bar_w + 5, by + bar_h),
                         FONT, FONT_SMALL - 0.05, (210, 210, 210))

        return out


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ArucoPosgDebugNode()
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
