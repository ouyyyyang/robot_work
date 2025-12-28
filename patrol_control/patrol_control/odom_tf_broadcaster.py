from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")

        self.declare_parameter("odom_topic", "/patrol_robot/odom")
        self.declare_parameter("odom_frame", "")
        self.declare_parameter("base_frame", "")

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._odom_frame_override = self.get_parameter("odom_frame").get_parameter_value().string_value
        self._base_frame_override = self.get_parameter("base_frame").get_parameter_value().string_value

        self._br = TransformBroadcaster(self)
        self._last_stamp_ns: Optional[int] = None

        self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
        self.get_logger().info(f"Subscribing: {odom_topic} (broadcasting odom->base_link TF)")

    def _on_odom(self, msg: Odometry) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if self._last_stamp_ns is not None and stamp_ns == self._last_stamp_ns:
            return
        self._last_stamp_ns = stamp_ns

        parent = (self._odom_frame_override or msg.header.frame_id).strip() or "odom"
        child = (self._base_frame_override or msg.child_frame_id).strip() or "base_link"

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(msg.pose.pose.position.x)
        t.transform.translation.y = float(msg.pose.pose.position.y)
        t.transform.translation.z = float(msg.pose.pose.position.z)
        t.transform.rotation = msg.pose.pose.orientation

        self._br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

