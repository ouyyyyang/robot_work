from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

import xml.etree.ElementTree as ET


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


class EnvironmentMarkers(Node):
    def __init__(self) -> None:
        super().__init__("environment_markers")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("marker_topic", "/patrol/markers")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("publish_rate", 2.0)

        self.declare_parameter("environment_sdf", "")
        self.declare_parameter("world_sdf", "")

        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        model_states_topic = self.get_parameter("model_states_topic").get_parameter_value().string_value
        rate = float(self.get_parameter("publish_rate").value)

        env_sdf = self.get_parameter("environment_sdf").get_parameter_value().string_value
        if not env_sdf:
            env_sdf = os.path.join(
                get_package_share_directory("patrol_bringup"),
                "models",
                "patrol_environment",
                "model.sdf",
            )

        world_sdf = self.get_parameter("world_sdf").get_parameter_value().string_value
        if not world_sdf:
            world_sdf = os.path.join(
                get_package_share_directory("patrol_bringup"),
                "worlds",
                "patrol_world.sdf",
            )

        self._static_markers = self._build_static_markers(env_sdf)
        self._obstacle_specs = self._load_obstacles_from_world(world_sdf)
        self._obstacle_poses: Dict[str, Tuple[float, float, float]] = {
            name: (spec.x, spec.y, spec.yaw) for name, spec in self._obstacle_specs.items()
        }

        self._pub = self.create_publisher(MarkerArray, marker_topic, 5)
        self.create_subscription(ModelStates, model_states_topic, self._on_model_states, 20)
        self.create_timer(1.0 / max(0.2, rate), self._tick)

        self.get_logger().info(f"Publishing: {marker_topic} frame={self._frame_id}")
        self.get_logger().info(f"Subscribing: {model_states_topic} (for obstacle poses)")

    @dataclass(frozen=True)
    class _ObstacleSpec:
        x: float
        y: float
        yaw: float
        sx: float
        sy: float
        sz: float

    def _load_obstacles_from_world(self, world_sdf: str) -> Dict[str, _ObstacleSpec]:
        root = ET.parse(world_sdf).getroot()
        world = root.find(".//world")
        if world is None:
            return {}

        out: Dict[str, EnvironmentMarkers._ObstacleSpec] = {}
        for m in world.findall("./model"):
            name = m.attrib.get("name", "")
            if not name.startswith("obstacle_"):
                continue
            pose_el = m.find("./pose")
            if pose_el is None or not pose_el.text:
                continue
            x, y, _z, _r, _p, yaw = _parse_pose(pose_el.text)

            size_el = m.find(".//link/collision/geometry/box/size")
            if size_el is None or not size_el.text:
                continue
            sx, sy, sz = [float(v) for v in size_el.text.strip().split()]
            out[name] = self._ObstacleSpec(x=x, y=y, yaw=yaw, sx=sx, sy=sy, sz=sz)
        return out

    def _build_static_markers(self, env_sdf: str) -> List[Marker]:
        root = ET.parse(env_sdf).getroot()
        model = root.find(".//model[@name='patrol_environment']")
        if model is None:
            raise RuntimeError("Could not find patrol_environment model in SDF")

        env_pose_el = model.find("./pose")
        env_x = env_y = env_yaw = 0.0
        if env_pose_el is not None and env_pose_el.text:
            env_x, env_y, _z, _r, _p, env_yaw = _parse_pose(env_pose_el.text)

        markers: List[Marker] = []

        # Walls
        wall_id = 0
        for link in model.findall("./link"):
            name = link.attrib.get("name", "")
            if not name.startswith("Wall_"):
                continue
            lpose_el = link.find("./pose")
            if lpose_el is None or not lpose_el.text:
                continue
            lx, ly, _lz, _r, _p, lyaw = _parse_pose(lpose_el.text)

            col_pose_el = link.find("./collision/pose")
            cz = 0.0
            if col_pose_el is not None and col_pose_el.text:
                _cx, _cy, cz, _cr, _cp, _cyaw = _parse_pose(col_pose_el.text)

            size_el = link.find("./collision/geometry/box/size")
            if size_el is None or not size_el.text:
                continue
            sx, sy, sz = [float(v) for v in size_el.text.strip().split()]

            rx, ry = _rot(lx, ly, env_yaw)
            wx = env_x + rx
            wy = env_y + ry
            wyaw = env_yaw + lyaw
            qx, qy, qz, qw = _yaw_to_quat(wyaw)

            mk = Marker()
            mk.header.frame_id = self._frame_id
            mk.ns = "walls"
            mk.id = wall_id
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x = float(wx)
            mk.pose.position.y = float(wy)
            mk.pose.position.z = float(cz)
            mk.pose.orientation.x = float(qx)
            mk.pose.orientation.y = float(qy)
            mk.pose.orientation.z = float(qz)
            mk.pose.orientation.w = float(qw)
            mk.scale.x = float(sx)
            mk.scale.y = float(sy)
            mk.scale.z = float(sz)
            mk.color.r = 0.5
            mk.color.g = 0.5
            mk.color.b = 0.5
            mk.color.a = 0.4
            markers.append(mk)
            wall_id += 1

        # Patrol points (spheres)
        point_id = 0
        for i in range(1, 6):
            pm = model.find(f".//model[@name='patrol_point_{i}']")
            if pm is None:
                continue
            pose_el = pm.find("./pose")
            if pose_el is None or not pose_el.text:
                continue
            px, py, pz, _r, _p, _yaw = _parse_pose(pose_el.text)
            rx, ry = _rot(px, py, env_yaw)
            wx = env_x + rx
            wy = env_y + ry
            wz = pz

            # Color from visual material ambient (red vs blue).
            ambient_el = pm.find(".//visual/material/ambient")
            r = g = b = 1.0
            if ambient_el is not None and ambient_el.text:
                vals = [float(v) for v in ambient_el.text.strip().split()]
                if len(vals) >= 3:
                    r, g, b = vals[0], vals[1], vals[2]

            mk = Marker()
            mk.header.frame_id = self._frame_id
            mk.ns = "patrol_points"
            mk.id = point_id
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = float(wx)
            mk.pose.position.y = float(wy)
            mk.pose.position.z = float(wz)
            mk.pose.orientation.w = 1.0
            mk.scale.x = 0.4
            mk.scale.y = 0.4
            mk.scale.z = 0.4
            mk.color.r = float(max(0.0, min(1.0, r)))
            mk.color.g = float(max(0.0, min(1.0, g)))
            mk.color.b = float(max(0.0, min(1.0, b)))
            mk.color.a = 0.9
            markers.append(mk)
            point_id += 1

        return markers

    def _on_model_states(self, msg: ModelStates) -> None:
        for name, pose in zip(msg.name, msg.pose):
            if name not in self._obstacle_specs:
                continue
            # Extract yaw (2D) from quaternion.
            q = pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self._obstacle_poses[name] = (float(pose.position.x), float(pose.position.y), float(yaw))

    def _tick(self) -> None:
        arr = MarkerArray()
        arr.markers.extend(self._static_markers)

        # Obstacles (dynamic)
        obs_id = 0
        for name in sorted(self._obstacle_specs.keys()):
            spec = self._obstacle_specs[name]
            x, y, yaw = self._obstacle_poses.get(name, (spec.x, spec.y, spec.yaw))
            qx, qy, qz, qw = _yaw_to_quat(yaw)

            mk = Marker()
            mk.header.frame_id = self._frame_id
            mk.ns = "obstacles"
            mk.id = obs_id
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x = float(x)
            mk.pose.position.y = float(y)
            mk.pose.position.z = 0.25
            mk.pose.orientation.x = float(qx)
            mk.pose.orientation.y = float(qy)
            mk.pose.orientation.z = float(qz)
            mk.pose.orientation.w = float(qw)
            mk.scale.x = float(spec.sx)
            mk.scale.y = float(spec.sy)
            mk.scale.z = float(spec.sz)
            mk.color.r = 1.0
            mk.color.g = 1.0
            mk.color.b = 0.0
            mk.color.a = 0.8
            arr.markers.append(mk)
            obs_id += 1

        self._pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = EnvironmentMarkers()
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
