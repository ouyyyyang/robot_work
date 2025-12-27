from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("wheel_joint_state_publisher")

        self.declare_parameter("odom_topic", "/patrol_robot/odom")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("wheel_separation", 0.32)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("left_joint", "left_wheel_joint")
        self.declare_parameter("right_joint", "right_wheel_joint")

        self._odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._joint_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )
        self._wheel_sep = float(self.get_parameter("wheel_separation").value)
        self._wheel_r = float(self.get_parameter("wheel_radius").value)
        self._left_joint = self.get_parameter("left_joint").get_parameter_value().string_value
        self._right_joint = self.get_parameter("right_joint").get_parameter_value().string_value

        self._last_stamp_ns: Optional[int] = None
        self._left_pos = 0.0
        self._right_pos = 0.0

        self._pub = self.create_publisher(JointState, self._joint_topic, 20)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 50)

        self.get_logger().info(f"Subscribing: {self._odom_topic}")
        self.get_logger().info(f"Publishing:  {self._joint_topic}")

    @staticmethod
    def _stamp_to_ns(msg: Odometry) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _on_odom(self, msg: Odometry) -> None:
        if self._wheel_r <= 0.0:
            return

        stamp_ns = self._stamp_to_ns(msg)
        if self._last_stamp_ns is None:
            self._last_stamp_ns = stamp_ns
            self._publish_joint_state(msg)
            return

        dt = (stamp_ns - self._last_stamp_ns) * 1e-9
        self._last_stamp_ns = stamp_ns
        if dt <= 0.0 or dt > 0.5:
            self._publish_joint_state(msg)
            return

        v = float(msg.twist.twist.linear.x)
        w = float(msg.twist.twist.angular.z)
        omega_l = (v - w * (self._wheel_sep / 2.0)) / self._wheel_r
        omega_r = (v + w * (self._wheel_sep / 2.0)) / self._wheel_r

        self._left_pos = (self._left_pos + omega_l * dt) % (2.0 * math.pi)
        self._right_pos = (self._right_pos + omega_r * dt) % (2.0 * math.pi)

        self._publish_joint_state(msg)

    def _publish_joint_state(self, odom: Odometry) -> None:
        msg = JointState()
        msg.header.stamp = odom.header.stamp
        msg.name = [self._left_joint, self._right_joint]
        msg.position = [float(self._left_pos), float(self._right_pos)]
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = WheelJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
