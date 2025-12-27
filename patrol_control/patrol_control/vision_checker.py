from __future__ import annotations

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
        self.declare_parameter("min_channel_value", 60)
        self.declare_parameter("min_pixel_fraction", 0.06)
        self.declare_parameter("publish_interval", 0.2)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value

        self._roi_size = int(self.get_parameter("roi_size").get_parameter_value().integer_value)
        self._dominance_ratio = float(
            self.get_parameter("dominance_ratio").get_parameter_value().double_value
        )
        self._min_channel = float(
            self.get_parameter("min_channel_value").get_parameter_value().integer_value
        )
        self._min_frac = float(self.get_parameter("min_pixel_fraction").value)
        self._publish_interval = float(self.get_parameter("publish_interval").value)
        self._last_pub_ns: int = 0

        self._pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)

        self._last_status: Optional[str] = None
        self.get_logger().info(f"Subscribing: {image_topic}")
        self.get_logger().info(f"Publishing:  {status_topic}")

    def _classify(
        self, rgb: np.ndarray
    ) -> Tuple[Optional[str], float, float, float, float, float]:
        h, w, _ = rgb.shape
        roi = max(10, min(self._roi_size, h, w))
        x0 = (w - roi) // 2
        y0 = (h - roi) // 2
        patch = rgb[y0 : y0 + roi, x0 : x0 + roi, :].astype(np.float32)

        r = patch[:, :, 0]
        g = patch[:, :, 1]
        b = patch[:, :, 2]

        mean_r = float(np.mean(r))
        mean_g = float(np.mean(g))
        mean_b = float(np.mean(b))

        dom = float(self._dominance_ratio)
        min_ch = float(self._min_channel)

        red_mask = (r >= dom * np.maximum(g, b)) & (r >= min_ch)
        blue_mask = (b >= dom * np.maximum(r, g)) & (b >= min_ch)

        total = int(r.size)
        if total <= 0:
            return None, 0.0, 0.0, mean_r, mean_g, mean_b
        red_frac = float(np.count_nonzero(red_mask) / total)
        blue_frac = float(np.count_nonzero(blue_mask) / total)

        if red_frac < self._min_frac and blue_frac < self._min_frac:
            return None, red_frac, blue_frac, mean_r, mean_g, mean_b

        status = "abnormal" if red_frac >= blue_frac else "normal"
        return status, red_frac, blue_frac, mean_r, mean_g, mean_b

    def _on_image(self, msg: Image) -> None:
        frame = _decode_rgb(msg)
        if frame is None:
            return

        status, red_frac, blue_frac, mean_r, mean_g, mean_b = self._classify(frame)
        if status is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if (
            status == self._last_status
            and self._publish_interval > 0.0
            and (now_ns - self._last_pub_ns) < int(self._publish_interval * 1e9)
        ):
            return

        self._last_status = status
        self._last_pub_ns = now_ns
        out = String()
        out.data = status
        self._pub.publish(out)
        self.get_logger().info(
            f"status={status} red={red_frac:.3f} blue={blue_frac:.3f} "
            f"(mean_r={mean_r:.1f}, mean_g={mean_g:.1f}, mean_b={mean_b:.1f})"
        )


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
