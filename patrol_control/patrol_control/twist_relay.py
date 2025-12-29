from __future__ import annotations

from typing import Dict, List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TwistRelay(Node):
    def __init__(self) -> None:
        super().__init__("twist_relay")

        self.declare_parameter("input_topics", ["/cmd_vel", "/cmd_vel_smoothed", "/cmd_vel_nav"])
        self.declare_parameter("output_topic", "/patrol_robot/cmd_vel")
        # Publish at a fixed rate to avoid diff_drive command timeout jitter when inputs are bursty.
        self.declare_parameter("publish_rate_hz", 20.0)
        # If no command received for this long, publish a zero Twist.
        self.declare_parameter("input_timeout", 0.5)

        in_topics: List[str] = [
            s.strip()
            for s in self.get_parameter("input_topics").get_parameter_value().string_array_value
            if s.strip()
        ]
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._publish_period = 1.0 / max(1.0, publish_rate_hz)
        timeout_s = float(self.get_parameter("input_timeout").value)
        self._timeout_ns = int(max(0.0, timeout_s) * 1e9)

        self._pub = self.create_publisher(Twist, out_topic, 20)

        self._in_topics = in_topics
        self._active_input: str = ""
        self._last_msg: Dict[str, Twist] = {}
        self._last_rx_ns: Dict[str, int] = {}
        for topic in in_topics:
            self.create_subscription(
                Twist,
                topic,
                lambda msg, t=topic: self._on_twist(msg, t),
                20,
            )

        self.get_logger().info(f"Publishing:  {out_topic}")
        self.get_logger().info(f"Subscribing: {', '.join(in_topics) if in_topics else '(none)'}")
        self.get_logger().info(
            f"publish_rate_hz={1.0 / self._publish_period:.1f} input_timeout={timeout_s:.2f}s"
        )

        self.create_timer(self._publish_period, self._tick)

    def _on_twist(self, msg: Twist, topic: str) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        self._last_msg[topic] = msg
        self._last_rx_ns[topic] = now_ns

    def _pick_active(self, now_ns: int) -> str:
        for topic in self._in_topics:
            rx_ns = self._last_rx_ns.get(topic)
            if rx_ns is None:
                continue
            if self._timeout_ns > 0 and (now_ns - rx_ns) > self._timeout_ns:
                continue
            return topic
        return ""

    def _tick(self) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        topic = self._pick_active(now_ns)
        if topic != self._active_input:
            self._active_input = topic
            if topic:
                self.get_logger().info(f"Relaying from: {topic}")
            else:
                self.get_logger().info("Relaying stopped (no fresh input)")

        if not topic:
            self._pub.publish(Twist())
            return

        msg = self._last_msg.get(topic)
        if msg is None:
            self._pub.publish(Twist())
            return
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
