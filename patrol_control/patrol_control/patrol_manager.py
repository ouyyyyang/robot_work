from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class PatrolPoint:
    name: str
    x: float
    y: float


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _parse_pose_text(text: str) -> Tuple[float, float, float]:
    parts = [p for p in text.strip().split() if p]
    if len(parts) < 3:
        raise ValueError(f"Invalid pose: {text!r}")
    x = float(parts[0])
    y = float(parts[1])
    yaw = float(parts[5]) if len(parts) >= 6 else 0.0
    return x, y, yaw


def _rotate(x: float, y: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def load_patrol_points_from_environment_sdf(path: str) -> List[PatrolPoint]:
    root = ET.parse(path).getroot()
    model = root.find(".//model[@name='patrol_environment']")
    if model is None:
        raise RuntimeError("Could not find model patrol_environment in SDF")

    pose_el = model.find("./pose")
    env_x = env_y = env_yaw = 0.0
    if pose_el is not None and pose_el.text:
        env_x, env_y, env_yaw = _parse_pose_text(pose_el.text)

    points: List[PatrolPoint] = []
    for i in range(1, 6):
        name = f"patrol_point_{i}"
        pm = model.find(f".//model[@name='{name}']")
        if pm is None:
            continue
        pp_pose_el = pm.find("./pose")
        if pp_pose_el is None or not pp_pose_el.text:
            continue
        px, py, _ = _parse_pose_text(pp_pose_el.text)
        rx, ry = _rotate(px, py, env_yaw)
        points.append(PatrolPoint(name=name, x=env_x + rx, y=env_y + ry))

    if not points:
        raise RuntimeError("No patrol_point_* found in environment SDF")
    return points


class PatrolManager(Node):
    def __init__(self) -> None:
        super().__init__("patrol_manager")

        self.declare_parameter("cmd_vel_topic", "/patrol_robot/cmd_vel")
        self.declare_parameter("odom_topic", "/patrol_robot/odom")
        self.declare_parameter("range_topic", "/patrol_robot/ultrasonic/range")
        self.declare_parameter("vision_status_topic", "/patrol/vision/status")
        self.declare_parameter("environment_sdf", "")

        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("max_linear", 0.35)
        self.declare_parameter("max_angular", 1.2)
        self.declare_parameter("k_linear", 0.6)
        self.declare_parameter("k_angular", 1.8)

        self.declare_parameter("obstacle_stop_distance", 0.45)
        self.declare_parameter("avoid_turn_angular", 1.0)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        range_topic = self.get_parameter("range_topic").get_parameter_value().string_value
        vision_topic = (
            self.get_parameter("vision_status_topic").get_parameter_value().string_value
        )

        env_sdf = self.get_parameter("environment_sdf").get_parameter_value().string_value
        if not env_sdf:
            env_sdf = os.path.join(
                get_package_share_directory("patrol_bringup"),
                "models",
                "patrol_environment",
                "model.sdf",
            )

        self._points = load_patrol_points_from_environment_sdf(env_sdf)
        self._idx = 0

        self._goal_tol = float(self.get_parameter("goal_tolerance").value)
        self._max_lin = float(self.get_parameter("max_linear").value)
        self._max_ang = float(self.get_parameter("max_angular").value)
        self._k_lin = float(self.get_parameter("k_linear").value)
        self._k_ang = float(self.get_parameter("k_angular").value)

        self._obs_stop = float(self.get_parameter("obstacle_stop_distance").value)
        self._avoid_w = float(self.get_parameter("avoid_turn_angular").value)

        self._pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(Range, range_topic, self._on_range, 20)
        self.create_subscription(String, vision_topic, self._on_vision, 10)

        self._pose: Optional[Tuple[float, float, float]] = None
        self._range: Optional[float] = None
        self._last_vision: Optional[str] = None
        self._waiting_vision = False
        self._current_result: Optional[str] = None

        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"Loaded patrol points: {[p.name for p in self._points]}")
        self.get_logger().info(f"cmd_vel: {cmd_vel_topic}")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        self._pose = (float(p.x), float(p.y), float(yaw))

    def _on_range(self, msg: Range) -> None:
        if msg.range <= 0.0:
            return
        self._range = float(msg.range)

    def _on_vision(self, msg: String) -> None:
        self._last_vision = msg.data.strip()
        if self._waiting_vision and self._last_vision in ("normal", "abnormal"):
            self._current_result = self._last_vision
            self._waiting_vision = False

    def _publish_cmd(self, v: float, w: float) -> None:
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self._pub_cmd.publish(t)

    def _tick(self) -> None:
        if self._idx >= len(self._points):
            self._publish_cmd(0.0, 0.0)
            return

        if self._pose is None:
            return

        if self._waiting_vision:
            self._publish_cmd(0.0, 0.0)
            if self._current_result is not None:
                point = self._points[self._idx]
                self.get_logger().info(f"[{point.name}] status={self._current_result}")
                self._current_result = None
                self._idx += 1
            return

        x, y, yaw = self._pose
        goal = self._points[self._idx]
        dx = goal.x - x
        dy = goal.y - y
        dist = math.hypot(dx, dy)

        if dist <= self._goal_tol:
            self.get_logger().info(f"Reached {goal.name}, waiting vision result...")
            self._waiting_vision = True
            self._current_result = None
            self._publish_cmd(0.0, 0.0)
            return

        if self._range is not None and self._range < self._obs_stop:
            self._publish_cmd(0.0, self._avoid_w)
            return

        target = math.atan2(dy, dx)
        err = _normalize_angle(target - yaw)

        w_cmd = max(-self._max_ang, min(self._max_ang, self._k_ang * err))
        v_cmd = min(self._max_lin, self._k_lin * dist)

        if abs(err) > 0.8:
            v_cmd *= 0.2
        elif abs(err) > 0.4:
            v_cmd *= 0.6

        self._publish_cmd(v_cmd, w_cmd)


def main() -> None:
    rclpy.init()
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

