from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class ScanToRange(Node):
    def __init__(self) -> None:
        super().__init__("scan_to_range")

        self.declare_parameter("scan_topic", "/patrol_robot/ultrasonic/scan")
        self.declare_parameter("range_topic", "/patrol_robot/ultrasonic/range")
        self.declare_parameter("default_fov", 0.2)

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        range_topic = self.get_parameter("range_topic").get_parameter_value().string_value
        self._default_fov = float(self.get_parameter("default_fov").value)

        self._pub = self.create_publisher(Range, range_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)

        self.get_logger().info(f"Subscribing: {scan_topic}")
        self.get_logger().info(f"Publishing:  {range_topic}")

    @staticmethod
    def _is_valid_range(r: float) -> bool:
        return math.isfinite(r) and r > 0.0

    def _pick_range(self, scan: LaserScan) -> Optional[float]:
        if not scan.ranges:
            return None

        center = scan.ranges[len(scan.ranges) // 2]
        if self._is_valid_range(center):
            return float(center)

        finite = [r for r in scan.ranges if self._is_valid_range(r)]
        if finite:
            return float(min(finite))

        if scan.range_max > 0.0:
            return float(scan.range_max)
        return None

    def _on_scan(self, scan: LaserScan) -> None:
        r = self._pick_range(scan)
        if r is None:
            return

        msg = Range()
        msg.header = scan.header
        msg.radiation_type = Range.ULTRASOUND

        fov = abs(float(scan.angle_max) - float(scan.angle_min))
        if not math.isfinite(fov) or fov <= 0.0:
            fov = self._default_fov
        msg.field_of_view = float(fov)

        msg.min_range = float(scan.range_min) if scan.range_min > 0.0 else 0.02
        msg.max_range = float(scan.range_max) if scan.range_max > msg.min_range else max(r, msg.min_range)

        if not self._is_valid_range(r):
            r = msg.max_range
        msg.range = float(max(msg.min_range, min(r, msg.max_range)))

        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScanToRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

