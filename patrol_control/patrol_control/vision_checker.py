from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _decode_rgb(image_msg: Image) -> Optional[np.ndarray]:
    if image_msg.height == 0 or image_msg.width == 0:
        return None

    if image_msg.encoding not in ("rgb8", "bgr8"):
        return None

    data = np.frombuffer(image_msg.data, dtype=np.uint8)
    expected_len = image_msg.height * image_msg.width * 3
    if data.size < expected_len:
        return None

    frame = data[:expected_len].reshape((image_msg.height, image_msg.width, 3))
    if image_msg.encoding == "bgr8":
        frame = frame[:, :, ::-1]
    return frame


class VisionChecker(Node):
    def __init__(self) -> None:
        super().__init__("vision_checker")

        self.declare_parameter("image_topic", "/patrol_robot/front_camera/image_raw")
        self.declare_parameter("status_topic", "/patrol/vision/status")
        self.declare_parameter("roi_size", 80)
        self.declare_parameter("dominance_ratio", 1.25)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value

        self._roi_size = int(self.get_parameter("roi_size").get_parameter_value().integer_value)
        self._dominance_ratio = float(
            self.get_parameter("dominance_ratio").get_parameter_value().double_value
        )

        self._pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)

        self._last_status: Optional[str] = None
        self.get_logger().info(f"Subscribing: {image_topic}")
        self.get_logger().info(f"Publishing:  {status_topic}")

    def _classify(self, rgb: np.ndarray) -> Tuple[str, float, float]:
        h, w, _ = rgb.shape
        roi = max(10, min(self._roi_size, h, w))
        x0 = (w - roi) // 2
        y0 = (h - roi) // 2
        patch = rgb[y0 : y0 + roi, x0 : x0 + roi, :].astype(np.float32)

        mean_r = float(np.mean(patch[:, :, 0]))
        mean_g = float(np.mean(patch[:, :, 1]))
        mean_b = float(np.mean(patch[:, :, 2]))

        r = mean_r + 1e-6
        b = mean_b + 1e-6
        ratio_rb = r / b

        if ratio_rb >= self._dominance_ratio:
            return "abnormal", mean_r, mean_b
        if (1.0 / ratio_rb) >= self._dominance_ratio:
            return "normal", mean_r, mean_b

        if mean_r + mean_b + mean_g < 30.0:
            return "unknown", mean_r, mean_b
        return "unknown", mean_r, mean_b

    def _on_image(self, msg: Image) -> None:
        frame = _decode_rgb(msg)
        if frame is None:
            return

        status, mean_r, mean_b = self._classify(frame)
        if status == self._last_status:
            return

        self._last_status = status
        out = String()
        out.data = status
        self._pub.publish(out)
        self.get_logger().info(f"status={status} (mean_r={mean_r:.1f}, mean_b={mean_b:.1f})")


def main() -> None:
    rclpy.init()
    node = VisionChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

