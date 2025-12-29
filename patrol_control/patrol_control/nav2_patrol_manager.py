from __future__ import annotations

import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

import tf2_geometry_msgs  # noqa: F401  (registers geometry conversions for tf2 in Python)


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _parse_pose(text: str) -> Tuple[float, float, float, float, float, float]:
    parts = [p for p in text.strip().split() if p]
    vals = [float(p) for p in parts]
    while len(vals) < 6:
        vals.append(0.0)
    return vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]


def _rot(x: float, y: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


@dataclass(frozen=True)
class PatrolPoint:
    index: int
    name: str
    x: float
    y: float
    yaw: float


class Nav2PatrolManager(Node):
    def __init__(self) -> None:
        super().__init__("nav2_patrol_manager")

        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("goal_frame", "map")
        # Patrol points are authored in Gazebo world coordinates. In this project, Gazebo diff-drive odom
        # is world-referenced by default, and SLAM provides `map -> odom`. So we default point_frame=odom
        # and transform to goal_frame (map) before sending goals.
        self.declare_parameter("point_frame", "odom")
        self.declare_parameter("environment_sdf", "")
        self.declare_parameter("dwell_time", 2.0)
        self.declare_parameter("loop_patrol", True)
        self.declare_parameter("server_timeout", 30.0)
        self.declare_parameter("goal_timeout", 120.0)

        self.declare_parameter("vision_status_topic", "/patrol/vision/status")
        self.declare_parameter("report_topic", "/patrol/inspection/report")

        self._action_name = (
            self.get_parameter("action_name").get_parameter_value().string_value
        )
        self._goal_frame = self.get_parameter("goal_frame").get_parameter_value().string_value
        self._point_frame = self.get_parameter("point_frame").get_parameter_value().string_value
        self._dwell_time = float(self.get_parameter("dwell_time").value)
        self._loop_patrol = bool(self.get_parameter("loop_patrol").value)
        self._server_timeout = float(self.get_parameter("server_timeout").value)
        self._goal_timeout = float(self.get_parameter("goal_timeout").value)

        env_sdf = self.get_parameter("environment_sdf").get_parameter_value().string_value
        if not env_sdf:
            env_sdf = os.path.join(
                get_package_share_directory("patrol_bringup"),
                "models",
                "patrol_environment",
                "model.sdf",
            )

        self._points = self._load_patrol_points(env_sdf)
        if not self._points:
            raise RuntimeError(f"No patrol points found in {env_sdf}")

        self._vision_status_topic = (
            self.get_parameter("vision_status_topic").get_parameter_value().string_value
        )
        report_topic = self.get_parameter("report_topic").get_parameter_value().string_value

        self._last_vision_status: Optional[str] = None
        self._dwell_status: Optional[str] = None
        self._dwell_point: Optional[str] = None

        self.create_subscription(String, self._vision_status_topic, self._on_vision, 10)
        self._report_pub = self.create_publisher(String, report_topic, 10)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._client = ActionClient(self, NavigateToPose, self._action_name)

        self._state = "WAIT_SERVER"
        self._idx = 0
        self._state_start = self.get_clock().now()
        self._dwell_until = self.get_clock().now()
        self._goal_start = self.get_clock().now()

        self._send_goal_future = None
        self._result_future = None
        self._active_goal_handle = None

        self._last_tf_warn_ns: int = 0

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"Nav2 action: {self._action_name} | points={len(self._points)} "
            f"goal_frame={self._goal_frame} point_frame={self._point_frame}"
        )
        self.get_logger().info(f"Vision status: {self._vision_status_topic}")
        self.get_logger().info(f"Inspection report: {report_topic}")

    @staticmethod
    def _load_patrol_points(env_sdf: str) -> List[PatrolPoint]:
        root = ET.parse(env_sdf).getroot()
        model = root.find(".//model[@name='patrol_environment']")
        if model is None:
            return []

        env_x = env_y = env_yaw = 0.0
        env_pose_el = model.find("./pose")
        if env_pose_el is not None and env_pose_el.text:
            env_x, env_y, _z, _r, _p, env_yaw = _parse_pose(env_pose_el.text)

        points: List[PatrolPoint] = []
        pat = re.compile(r"^patrol_point_(\d+)$")
        for pm in model.findall("./model"):
            name = pm.attrib.get("name", "")
            m = pat.match(name)
            if m is None:
                continue
            idx = int(m.group(1))
            pose_el = pm.find("./pose")
            if pose_el is None or not pose_el.text:
                continue
            px, py, _pz, _r, _p, yaw = _parse_pose(pose_el.text)
            rx, ry = _rot(px, py, env_yaw)
            wx = env_x + rx
            wy = env_y + ry
            points.append(PatrolPoint(index=idx, name=name, x=wx, y=wy, yaw=yaw + env_yaw))

        points.sort(key=lambda p: p.index)
        return points

    def _on_vision(self, msg: String) -> None:
        status = msg.data.strip()
        if status not in ("normal", "abnormal"):
            return
        self._last_vision_status = status
        if self._state == "DWELL" and self._dwell_point is not None:
            self._dwell_status = status

    def _tick(self) -> None:
        now = self.get_clock().now()

        if self._state == "WAIT_SERVER":
            if self._client.wait_for_server(timeout_sec=0.0):
                self.get_logger().info("Nav2 action server is ready")
                self._state = "SEND_GOAL"
                self._state_start = now
                return
            if (now - self._state_start).nanoseconds * 1e-9 > self._server_timeout:
                self.get_logger().error(
                    f"Timeout waiting for Nav2 action server: {self._action_name}"
                )
                self._state_start = now
            return

        if self._state == "SEND_GOAL":
            point = self._points[self._idx]
            goal = self._build_goal(point)
            if goal is None:
                return
            self._goal_start = now
            self._send_goal_future = self._client.send_goal_async(goal)
            self._state = "WAIT_GOAL_RESPONSE"
            self._state_start = now
            self.get_logger().info(
                f"Sending goal: {point.name} (x={point.x:.2f}, y={point.y:.2f}, yaw={point.yaw:.2f})"
            )
            return

        if self._state == "WAIT_GOAL_RESPONSE":
            if self._send_goal_future is None or not self._send_goal_future.done():
                return
            goal_handle = self._send_goal_future.result()
            self._active_goal_handle = goal_handle
            if not goal_handle.accepted:
                self.get_logger().warn(
                    f"Goal rejected: {self._points[self._idx].name}, skipping"
                )
                self._advance_index()
                self._state = "SEND_GOAL"
                self._state_start = now
                return
            self._result_future = goal_handle.get_result_async()
            self._state = "WAIT_RESULT"
            self._state_start = now
            self.get_logger().info(f"Goal accepted: {self._points[self._idx].name}")
            return

        if self._state == "WAIT_RESULT":
            if (now - self._goal_start).nanoseconds * 1e-9 > self._goal_timeout:
                if self._active_goal_handle is not None:
                    self.get_logger().warn(
                        f"Goal timeout, cancelling: {self._points[self._idx].name}"
                    )
                    self._active_goal_handle.cancel_goal_async()
                self._state = "DWELL"
                self._enter_dwell(now)
                return

            if self._result_future is None or not self._result_future.done():
                return

            result = self._result_future.result()
            code = int(result.status)
            self.get_logger().info(
                f"Goal result: {self._points[self._idx].name} status={code}"
            )
            self._state = "DWELL"
            self._enter_dwell(now)
            return

        if self._state == "DWELL":
            if now < self._dwell_until:
                return
            self._finalize_report()
            self._advance_index()
            self._state = "SEND_GOAL"
            self._state_start = now
            return

    def _enter_dwell(self, now) -> None:
        self._dwell_point = self._points[self._idx].name
        self._dwell_status = None
        self._dwell_until = now + Duration(seconds=max(0.0, self._dwell_time))
        self.get_logger().info(
            f"Dwell at {self._dwell_point} for {max(0.0, self._dwell_time):.1f}s"
        )

    def _finalize_report(self) -> None:
        if self._dwell_point is None:
            return
        status = self._dwell_status or self._last_vision_status or "normal"
        msg = String()
        msg.data = f"{self._dwell_point}:{status}"
        self._report_pub.publish(msg)
        self.get_logger().info(f"Inspection report: {msg.data}")
        self._dwell_point = None
        self._dwell_status = None

    def _advance_index(self) -> None:
        self._idx += 1
        if self._idx >= len(self._points):
            if self._loop_patrol:
                self._idx = 0
            else:
                self.get_logger().info("Patrol finished (loop_patrol=false), stopping")
                self._idx = len(self._points) - 1
                self._state = "IDLE"

    def _build_goal(self, point: PatrolPoint) -> Optional[NavigateToPose.Goal]:
        now_msg = self.get_clock().now().to_msg()

        pose = PoseStamped()
        # Use "latest available" TF for frame transforms (stamp=0), then overwrite to now.
        pose.header.stamp = Time().to_msg()
        pose.header.frame_id = self._point_frame
        pose.pose.position.x = float(point.x)
        pose.pose.position.y = float(point.y)
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quat(point.yaw)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        if self._point_frame != self._goal_frame:
            try:
                pose = self._tf_buffer.transform(
                    pose,
                    self._goal_frame,
                    timeout=Duration(seconds=0.2),
                )
            except TransformException as exc:
                self._warn_tf_throttled(
                    f"Waiting for TF: {self._goal_frame} <- {self._point_frame} ({exc})"
                )
                return None
            except Exception as exc:  # tf2 may raise TypeException if conversions aren't registered
                self._warn_tf_throttled(
                    f"TF transform failed: {self._goal_frame} <- {self._point_frame} ({exc})"
                )
                return None

        pose.header.stamp = now_msg

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _warn_tf_throttled(self, text: str, period_s: float = 1.0) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        if now_ns - self._last_tf_warn_ns < int(period_s * 1e9):
            return
        self._last_tf_warn_ns = now_ns
        self.get_logger().warn(text)


def main() -> None:
    rclpy.init()
    node = Nav2PatrolManager()
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
