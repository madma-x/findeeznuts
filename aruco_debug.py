#!/usr/bin/env python3
"""
ArUco debug viewer.
Subscribes to the camera topic and tries several common ArUco dictionaries.
Draws every detected marker on screen and prints its dictionary name + ID.
Use this to figure out which dictionary your physical tags belong to.
Press 'q' to quit.
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# ----- dictionaries to try -------------------------------------------------
DICTS = {
    "4x4_50":   cv2.aruco.DICT_4X4_50,
    "4x4_100":  cv2.aruco.DICT_4X4_100,
    "4x4_250":  cv2.aruco.DICT_4X4_250,
    "5x5_50":   cv2.aruco.DICT_5X5_50,
    "5x5_100":  cv2.aruco.DICT_5X5_100,
    "5x5_250":  cv2.aruco.DICT_5X5_250,
    "6x6_50":   cv2.aruco.DICT_6X6_50,
    "6x6_100":  cv2.aruco.DICT_6X6_100,
    "6x6_250":  cv2.aruco.DICT_6X6_250,   # <-- what the node uses (dict id=10)
    "7x7_50":   cv2.aruco.DICT_7X7_50,
}

# Pre-build detectors with relaxed params for better detection rate
params = cv2.aruco.DetectorParameters()
params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
params.adaptiveThreshWinSizeMin  = 3
params.adaptiveThreshWinSizeMax  = 53
params.adaptiveThreshWinSizeStep = 4
params.minMarkerPerimeterRate    = 0.02
params.maxMarkerPerimeterRate    = 4.0
params.polygonalApproxAccuracyRate = 0.05

DETECTORS = {
    name: cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(did), params
    )
    for name, did in DICTS.items()
}

# Colors per dictionary (BGR)
COLORS = [
    (0, 255, 0), (0, 200, 255), (255, 0, 0), (0, 0, 255),
    (255, 255, 0), (255, 0, 255), (128, 255, 0), (0, 128, 255),
    (255, 128, 0), (128, 0, 255),
]
DICT_COLORS = {name: COLORS[i % len(COLORS)] for i, name in enumerate(DICTS)}


class ArucoDebugViewer(Node):
    def __init__(self, topic: str):
        super().__init__("aruco_debug_viewer")
        self.bridge = CvBridge()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(Image, topic, self._cb, qos)
        self.get_logger().info(f"Listening on {topic}  — press 'q' in the window to quit")

    def _cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        grey  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        overlay = frame.copy()
        found_any = False

        for name, detector in DETECTORS.items():
            corners, ids, _ = detector.detectMarkers(grey)
            if ids is None or len(ids) == 0:
                continue
            found_any = True
            color = DICT_COLORS[name]
            cv2.aruco.drawDetectedMarkers(overlay, corners, ids, color)
            for i, tag_id in enumerate(ids.flatten()):
                c = corners[i][0]
                cx = int(c[:, 0].mean())
                cy = int(c[:, 1].mean())
                cv2.putText(overlay, f"{name} #{tag_id}", (cx - 30, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            print(f"[{name}]  IDs detected: {sorted(ids.flatten().tolist())}")

        if not found_any:
            cv2.putText(overlay, "No tags detected in any dict",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2, cv2.LINE_AA)

        # Legend
        y = overlay.shape[0] - 10 - 18 * len(DICTS)
        for name, color in DICT_COLORS.items():
            cv2.putText(overlay, name, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)
            y += 18

        cv2.imshow("ArUco Debug", overlay)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/camera/image_raw"
    rclpy.init()
    node = ArucoDebugViewer(topic)
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
