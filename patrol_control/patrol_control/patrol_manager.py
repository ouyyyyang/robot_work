from __future__ import annotations

import heapq
import math
import os
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

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


@dataclass(frozen=True)
class _Rect2D:
    cx: float
    cy: float
    c: float
    s: float
    hx: float
    hy: float

    def aabb(self) -> Tuple[float, float, float, float]:
        corners = (
            (self.hx, self.hy),
            (self.hx, -self.hy),
            (-self.hx, -self.hy),
            (-self.hx, self.hy),
        )
        xs = []
        ys = []
        for x, y in corners:
            wx = self.cx + self.c * x - self.s * y
            wy = self.cy + self.s * x + self.c * y
            xs.append(wx)
            ys.append(wy)
        return min(xs), min(ys), max(xs), max(ys)

    def contains(self, x: float, y: float) -> bool:
        dx = x - self.cx
        dy = y - self.cy
        rx = self.c * dx + self.s * dy
        ry = -self.s * dx + self.c * dy
        return (abs(rx) <= self.hx) and (abs(ry) <= self.hy)


class _WallGrid:
    def __init__(
        self,
        walls: List[_Rect2D],
        *,
        resolution: float,
        margin: float,
    ) -> None:
        if resolution <= 0.0:
            raise ValueError("resolution must be > 0")
        if margin < 0.0:
            raise ValueError("margin must be >= 0")

        self._res = float(resolution)
        self._walls = walls

        min_x = float("inf")
        min_y = float("inf")
        max_x = float("-inf")
        max_y = float("-inf")
        for wall in walls:
            ax0, ay0, ax1, ay1 = wall.aabb()
            min_x = min(min_x, ax0)
            min_y = min(min_y, ay0)
            max_x = max(max_x, ax1)
            max_y = max(max_y, ay1)

        if not math.isfinite(min_x) or not math.isfinite(min_y):
            raise RuntimeError("No walls found to build a planning grid")

        self._min_x = min_x - margin
        self._min_y = min_y - margin
        self._max_x = max_x + margin
        self._max_y = max_y + margin
        self._nx = int(math.ceil((self._max_x - self._min_x) / self._res))
        self._ny = int(math.ceil((self._max_y - self._min_y) / self._res))

        if self._nx <= 1 or self._ny <= 1:
            raise RuntimeError("Planning grid too small")

        self._occ = [False] * (self._nx * self._ny)
        self._build_occupancy()
        self._free_mask = self._select_track_component()

    @property
    def resolution(self) -> float:
        return self._res

    def _cell_id(self, i: int, j: int) -> int:
        return j * self._nx + i

    def _ij(self, cell_id: int) -> Tuple[int, int]:
        return (cell_id % self._nx, cell_id // self._nx)

    def _cell_center(self, i: int, j: int) -> Tuple[float, float]:
        x = self._min_x + (i + 0.5) * self._res
        y = self._min_y + (j + 0.5) * self._res
        return x, y

    def _build_occupancy(self) -> None:
        for wall in self._walls:
            ax0, ay0, ax1, ay1 = wall.aabb()
            i0 = max(0, int((ax0 - self._min_x) / self._res) - 1)
            j0 = max(0, int((ay0 - self._min_y) / self._res) - 1)
            i1 = min(self._nx - 1, int((ax1 - self._min_x) / self._res) + 1)
            j1 = min(self._ny - 1, int((ay1 - self._min_y) / self._res) + 1)
            for j in range(j0, j1 + 1):
                for i in range(i0, i1 + 1):
                    x, y = self._cell_center(i, j)
                    if wall.contains(x, y):
                        self._occ[self._cell_id(i, j)] = True

    def _neighbors4(self, cell_id: int) -> Iterable[int]:
        i, j = self._ij(cell_id)
        if i + 1 < self._nx:
            yield cell_id + 1
        if i - 1 >= 0:
            yield cell_id - 1
        if j + 1 < self._ny:
            yield cell_id + self._nx
        if j - 1 >= 0:
            yield cell_id - self._nx

    def _select_track_component(self) -> List[bool]:
        labels = [-1] * (self._nx * self._ny)
        comp_start: List[int] = []
        comp_size: List[int] = []
        comp_touch: List[bool] = []

        comp_id = 0
        for j in range(self._ny):
            for i in range(self._nx):
                cid = self._cell_id(i, j)
                if self._occ[cid] or labels[cid] != -1:
                    continue
                q = deque([cid])
                labels[cid] = comp_id
                size = 0
                touches = False
                while q:
                    cur = q.popleft()
                    size += 1
                    ci, cj = self._ij(cur)
                    if ci == 0 or cj == 0 or ci == self._nx - 1 or cj == self._ny - 1:
                        touches = True
                    for nb in self._neighbors4(cur):
                        if self._occ[nb] or labels[nb] != -1:
                            continue
                        labels[nb] = comp_id
                        q.append(nb)
                comp_start.append(cid)
                comp_size.append(size)
                comp_touch.append(touches)
                comp_id += 1

        if not comp_start:
            return [False] * (self._nx * self._ny)

        dist = [-1] * (self._nx * self._ny)

        def farthest(start: int, label: int) -> Tuple[int, int]:
            q = deque([start])
            dist[start] = 0
            visited = [start]
            far = start
            while q:
                cur = q.popleft()
                if dist[cur] > dist[far]:
                    far = cur
                for nb in self._neighbors4(cur):
                    if labels[nb] != label or dist[nb] != -1:
                        continue
                    dist[nb] = dist[cur] + 1
                    visited.append(nb)
                    q.append(nb)
            d_far = dist[far]
            for v in visited:
                dist[v] = -1
            return far, d_far

        best_label: Optional[int] = None
        best_diam = -1
        for label, start in enumerate(comp_start):
            if comp_touch[label]:
                continue
            a, _ = farthest(start, label)
            _b, diam = farthest(a, label)
            if diam > best_diam:
                best_diam = diam
                best_label = label

        if best_label is None:
            best_label = max(range(len(comp_size)), key=lambda k: comp_size[k])

        free_mask = [False] * (self._nx * self._ny)
        for idx, label in enumerate(labels):
            if label == best_label:
                free_mask[idx] = True
        return free_mask

    def _world_to_cell(self, x: float, y: float) -> Optional[int]:
        i = int((x - self._min_x) / self._res)
        j = int((y - self._min_y) / self._res)
        if not (0 <= i < self._nx and 0 <= j < self._ny):
            return None
        return self._cell_id(i, j)

    def _cell_to_world(self, cell_id: int) -> Tuple[float, float]:
        i, j = self._ij(cell_id)
        return self._cell_center(i, j)

    def _is_free(self, cell_id: int) -> bool:
        return (not self._occ[cell_id]) and self._free_mask[cell_id]

    def _nearest_free(self, cell_id: int, *, max_steps: int = 8000) -> Optional[int]:
        if self._is_free(cell_id):
            return cell_id
        q = deque([cell_id])
        seen = {cell_id}
        steps = 0
        while q and steps < max_steps:
            cur = q.popleft()
            steps += 1
            for nb in self._neighbors4(cur):
                if nb in seen:
                    continue
                if self._is_free(nb):
                    return nb
                seen.add(nb)
                q.append(nb)
        return None

    def plan(self, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        start_cell = self._world_to_cell(*start_xy)
        goal_cell = self._world_to_cell(*goal_xy)
        if start_cell is None or goal_cell is None:
            return None

        start_cell = self._nearest_free(start_cell)
        goal_cell = self._nearest_free(goal_cell)
        if start_cell is None or goal_cell is None:
            return None

        if start_cell == goal_cell:
            x, y = self._cell_to_world(goal_cell)
            return [(x, y)]

        def h(cell: int) -> float:
            i, j = self._ij(cell)
            gi, gj = self._ij(goal_cell)
            return math.hypot(gi - i, gj - j)

        def neighbors8(cell: int) -> Iterable[Tuple[int, float]]:
            i, j = self._ij(cell)
            for di, dj, cost in (
                (1, 0, 1.0),
                (-1, 0, 1.0),
                (0, 1, 1.0),
                (0, -1, 1.0),
                (1, 1, math.sqrt(2.0)),
                (1, -1, math.sqrt(2.0)),
                (-1, 1, math.sqrt(2.0)),
                (-1, -1, math.sqrt(2.0)),
            ):
                ni = i + di
                nj = j + dj
                if not (0 <= ni < self._nx and 0 <= nj < self._ny):
                    continue
                nb = self._cell_id(ni, nj)
                if not self._is_free(nb):
                    continue
                if abs(di) == 1 and abs(dj) == 1:
                    if not self._is_free(self._cell_id(i + di, j)):
                        continue
                    if not self._is_free(self._cell_id(i, j + dj)):
                        continue
                yield nb, cost

        open_heap: List[Tuple[float, float, int]] = []
        heapq.heappush(open_heap, (h(start_cell), 0.0, start_cell))
        came_from: Dict[int, int] = {start_cell: start_cell}
        g_score: Dict[int, float] = {start_cell: 0.0}
        closed: set[int] = set()

        while open_heap:
            _f, g, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            if cur == goal_cell:
                break
            closed.add(cur)
            for nb, step_cost in neighbors8(cur):
                tentative = g + step_cost
                if tentative >= g_score.get(nb, float("inf")):
                    continue
                g_score[nb] = tentative
                came_from[nb] = cur
                heapq.heappush(open_heap, (tentative + h(nb), tentative, nb))

        if goal_cell not in came_from:
            return None

        # Reconstruct (cell ids)
        cells: List[int] = []
        cur = goal_cell
        while cur != came_from[cur]:
            cells.append(cur)
            cur = came_from[cur]
        cells.append(start_cell)
        cells.reverse()

        return [self._cell_to_world(c) for c in cells]


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


def load_wall_rects_from_environment_sdf(
    path: str, *, inflation: float
) -> List[_Rect2D]:
    root = ET.parse(path).getroot()
    model = root.find(".//model[@name='patrol_environment']")
    if model is None:
        raise RuntimeError("Could not find model patrol_environment in SDF")

    env_x = env_y = env_yaw = 0.0
    pose_el = model.find("./pose")
    if pose_el is not None and pose_el.text:
        env_x, env_y, env_yaw = _parse_pose_text(pose_el.text)

    walls: List[_Rect2D] = []
    for link in model.findall("./link"):
        name = link.attrib.get("name", "")
        if not name.startswith("Wall_"):
            continue
        lp = link.find("./pose")
        if lp is None or not lp.text:
            continue
        lx, ly, lyaw = _parse_pose_text(lp.text)

        col = link.find("./collision")
        if col is None:
            continue
        size_el = col.find(".//box/size")
        if size_el is None or not size_el.text:
            continue
        sx, sy, *_ = [float(v) for v in size_el.text.strip().split()]

        rx, ry = _rotate(lx, ly, env_yaw)
        wx = env_x + rx
        wy = env_y + ry
        wyaw = env_yaw + lyaw
        hx = sx / 2.0 + inflation
        hy = sy / 2.0 + inflation
        walls.append(_Rect2D(cx=wx, cy=wy, c=math.cos(wyaw), s=math.sin(wyaw), hx=hx, hy=hy))

    if not walls:
        raise RuntimeError("No Wall_* links found in environment SDF")
    return walls


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

        self.declare_parameter("use_path_planner", True)
        self.declare_parameter("grid_resolution", 0.10)
        self.declare_parameter("grid_margin", 1.0)
        self.declare_parameter("wall_inflation", 0.25)
        self.declare_parameter("plan_interval", 1.0)
        self.declare_parameter("lookahead_distance", 0.7)
        self.declare_parameter("waypoint_tolerance", 0.35)

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

        self._use_planner = bool(self.get_parameter("use_path_planner").value)
        self._plan_interval = float(self.get_parameter("plan_interval").value)
        self._lookahead = float(self.get_parameter("lookahead_distance").value)
        self._wp_tol = float(self.get_parameter("waypoint_tolerance").value)

        self._obs_stop = float(self.get_parameter("obstacle_stop_distance").value)
        self._avoid_w = float(self.get_parameter("avoid_turn_angular").value)

        self._grid: Optional[_WallGrid] = None
        if self._use_planner:
            try:
                inflation = float(self.get_parameter("wall_inflation").value)
                grid_res = float(self.get_parameter("grid_resolution").value)
                grid_margin = float(self.get_parameter("grid_margin").value)
                walls = load_wall_rects_from_environment_sdf(env_sdf, inflation=inflation)
                self._grid = _WallGrid(walls, resolution=grid_res, margin=grid_margin)
                self.get_logger().info(
                    f"Path planner enabled: res={grid_res:.2f}m inflation={inflation:.2f}m"
                )
            except Exception as exc:
                self._use_planner = False
                self._grid = None
                self.get_logger().error(f"Failed to build wall planner grid: {exc}")

        self._pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(Range, range_topic, self._on_range, 20)
        self.create_subscription(String, vision_topic, self._on_vision, 10)

        self._pose: Optional[Tuple[float, float, float]] = None
        self._range: Optional[float] = None
        self._last_vision: Optional[str] = None
        self._waiting_vision = False
        self._current_result: Optional[str] = None
        self._path: Optional[List[Tuple[float, float]]] = None
        self._path_idx: int = 0
        self._path_goal_idx: Optional[int] = None
        self._last_plan_time = self.get_clock().now()

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
                self._path = None
                self._path_goal_idx = None
                self._path_idx = 0
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
            self._path = None
            self._path_goal_idx = None
            self._path_idx = 0
            return

        if self._range is not None and self._range < self._obs_stop:
            self._publish_cmd(0.0, self._avoid_w)
            self._path = None
            self._path_goal_idx = None
            self._path_idx = 0
            return

        target_x = goal.x
        target_y = goal.y

        if self._use_planner and self._grid is not None:
            now = self.get_clock().now()
            need_plan = (
                self._path is None
                or self._path_goal_idx != self._idx
                or (now - self._last_plan_time).nanoseconds * 1e-9 >= max(0.2, self._plan_interval)
            )
            if need_plan:
                self._path = self._grid.plan((x, y), (goal.x, goal.y))
                self._path_idx = 0
                self._path_goal_idx = self._idx
                self._last_plan_time = now

            if self._path:
                while self._path_idx < len(self._path) - 1:
                    px, py = self._path[self._path_idx]
                    if math.hypot(px - x, py - y) > self._wp_tol:
                        break
                    self._path_idx += 1
                lookahead_steps = max(1, int(self._lookahead / self._grid.resolution))
                target_idx = min(self._path_idx + lookahead_steps, len(self._path) - 1)
                target_x, target_y = self._path[target_idx]

        target = math.atan2(target_y - y, target_x - x)
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
