from __future__ import annotations

import math
from typing import List

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
from rclpy.node import Node


class ObstacleController(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_controller")

        # Gazebo classic + gazebo_ros (ROS2) commonly exposes this service as `/set_entity_state`.
        # Some setups may namespace it as `/gazebo/set_entity_state`, so we auto-detect when unset.
        self.declare_parameter("service_name", "")
        self.declare_parameter("service_candidates", ["/set_entity_state", "/gazebo/set_entity_state"])
        self.declare_parameter("obstacles", ["obstacle_1"])
        self.declare_parameter("point_a_x", 0.0)
        self.declare_parameter("point_a_y", 0.0)
        self.declare_parameter("point_b_x", 1.0)
        self.declare_parameter("point_b_y", 0.0)
        self.declare_parameter("speed", 0.3)  # m/s
        self.declare_parameter("z", 0.25)
        self.declare_parameter("rate_hz", 5.0)

        requested_service = (
            self.get_parameter("service_name").get_parameter_value().string_value.strip()
        )
        candidates = [
            s.strip()
            for s in self.get_parameter("service_candidates").get_parameter_value().string_array_value
            if s.strip()
        ]
        self._service_candidates = [requested_service] if requested_service else (candidates or ["/set_entity_state"])
        self._obstacles: List[str] = [
            s.strip()
            for s in self.get_parameter("obstacles").get_parameter_value().string_array_value
            if s.strip()
        ]
        self._ax = float(self.get_parameter("point_a_x").get_parameter_value().double_value)
        self._ay = float(self.get_parameter("point_a_y").get_parameter_value().double_value)
        self._bx = float(self.get_parameter("point_b_x").get_parameter_value().double_value)
        self._by = float(self.get_parameter("point_b_y").get_parameter_value().double_value)
        self._speed = abs(float(self.get_parameter("speed").get_parameter_value().double_value))
        self._z = float(self.get_parameter("z").get_parameter_value().double_value)
        rate_hz = float(self.get_parameter("rate_hz").get_parameter_value().double_value)

        dx = self._bx - self._ax
        dy = self._by - self._ay
        self._segment_len = math.hypot(dx, dy)
        self._period = 2.0 * self._segment_len

        self._phases = [0.0 for _ in self._obstacles]  # distance along [0, 2*L)
        self._last_time = self.get_clock().now()

        self._clients = [
            (name, self.create_client(SetEntityState, name)) for name in self._service_candidates
        ]
        self._active_client = None
        self._active_service_name = ""
        self._last_wait_log_ns: int = 0
        self.get_logger().info(
            f"Waiting for service: {', '.join(self._service_candidates) or '(none)'}"
        )
        self.get_logger().info(
            f"Obstacles: {', '.join(self._obstacles) or '(none)'} | "
            f"A=({self._ax:.2f}, {self._ay:.2f}) B=({self._bx:.2f}, {self._by:.2f}) "
            f"speed={self._speed:.2f} m/s"
        )

        self.create_timer(1.0 / max(0.1, rate_hz), self._tick)

    def _tick(self) -> None:
        if self._active_client is None or not self._active_client.service_is_ready():
            self._active_client = None
            self._active_service_name = ""
            for name, client in self._clients:
                if client.service_is_ready():
                    self._active_client = client
                    self._active_service_name = name
                    self.get_logger().info(f"Using service: {name}")
                    break

        if self._active_client is None:
            # Non-blocking wait so the node can still shutdown cleanly when Gazebo isn't ready.
            for _name, client in self._clients:
                client.wait_for_service(timeout_sec=0.0)
            now_ns = int(self.get_clock().now().nanoseconds)
            if now_ns - self._last_wait_log_ns > int(1e9):
                self._last_wait_log_ns = now_ns
                self.get_logger().info(
                    f"Waiting for service: {', '.join(self._service_candidates)}"
                )
            return

        if not self._obstacles:
            return
        if self._segment_len <= 1e-6 or self._period <= 1e-6:
            return

        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 0.0
        self._last_time = now

        step = self._speed * dt
        for idx, name in enumerate(self._obstacles):
            phase = (self._phases[idx] + step) % self._period
            self._phases[idx] = phase

            if phase <= self._segment_len:
                s = phase / self._segment_len
            else:
                s = 2.0 - (phase / self._segment_len)

            x = self._ax + s * (self._bx - self._ax)
            y = self._ay + s * (self._by - self._ay)

            state = EntityState()
            state.name = name
            state.pose = Pose()
            state.pose.position.x = float(x)
            state.pose.position.y = float(y)
            state.pose.position.z = float(self._z)
            state.pose.orientation.w = 1.0
            state.reference_frame = "world"

            req = SetEntityState.Request()
            req.state = state
            if self._active_client is not None:
                self._active_client.call_async(req)


def main() -> None:
    rclpy.init()
    node = ObstacleController()
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
