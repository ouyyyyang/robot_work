from __future__ import annotations

from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TwistRelay(Node):
    def __init__(self) -> None:
        super().__init__("twist_relay")

        self.declare_parameter("input_topics", ["/cmd_vel", "/cmd_vel_smoothed", "/cmd_vel_nav"])
        self.declare_parameter("output_topic", "/patrol_robot/cmd_vel")

        in_topics: List[str] = [
            s.strip()
            for s in self.get_parameter("input_topics").get_parameter_value().string_array_value
            if s.strip()
        ]
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self._pub = self.create_publisher(Twist, out_topic, 20)

        self._active_input: str = ""
        for topic in in_topics:
            self.create_subscription(
                Twist,
                topic,
                lambda msg, t=topic: self._on_twist(msg, t),
                20,
            )

        self.get_logger().info(f"Publishing:  {out_topic}")
        self.get_logger().info(f"Subscribing: {', '.join(in_topics) if in_topics else '(none)'}")

    def _on_twist(self, msg: Twist, topic: str) -> None:
        if topic != self._active_input:
            self._active_input = topic
            self.get_logger().info(f"Relaying from: {topic}")
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
