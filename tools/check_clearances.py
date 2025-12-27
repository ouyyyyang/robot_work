#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import pathlib
import xml.etree.ElementTree as ET


def _parse_pose(text: str) -> tuple[float, float, float]:
    parts = [p for p in text.strip().split() if p]
    if len(parts) < 2:
        raise ValueError(f"Invalid pose: {text!r}")
    x = float(parts[0])
    y = float(parts[1])
    yaw = float(parts[5]) if len(parts) >= 6 else 0.0
    return x, y, yaw


def _rect_vertices(cx: float, cy: float, yaw: float, hx: float, hy: float) -> list[tuple[float, float]]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    corners = ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))
    return [(cx + c * x - s * y, cy + s * x + c * y) for x, y in corners]


def _dot(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _project(poly: list[tuple[float, float]], axis: tuple[float, float]) -> tuple[float, float]:
    values = [_dot(p, axis) for p in poly]
    return min(values), max(values)


def _overlap_1d(a: tuple[float, float], b: tuple[float, float]) -> bool:
    return not (a[1] < b[0] or b[1] < a[0])


def _polygons_overlap(poly1: list[tuple[float, float]], poly2: list[tuple[float, float]]) -> bool:
    for poly in (poly1, poly2):
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            ex = x2 - x1
            ey = y2 - y1
            axis = (-ey, ex)
            if not _overlap_1d(_project(poly1, axis), _project(poly2, axis)):
                return False
    return True


def _point_segment_distance(
    p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]
) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    ab2 = abx * abx + aby * aby
    if ab2 <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = (apx * abx + apy * aby) / ab2
    t = max(0.0, min(1.0, t))
    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy)


def _polygons_distance(poly1: list[tuple[float, float]], poly2: list[tuple[float, float]]) -> float:
    if _polygons_overlap(poly1, poly2):
        return 0.0
    best = float("inf")
    for p in poly1:
        for i in range(len(poly2)):
            best = min(best, _point_segment_distance(p, poly2[i], poly2[(i + 1) % len(poly2)]))
    for p in poly2:
        for i in range(len(poly1)):
            best = min(best, _point_segment_distance(p, poly1[i], poly1[(i + 1) % len(poly1)]))
    return best


def _load_walls(env_sdf: pathlib.Path) -> list[tuple[str, float, float, float, float, float]]:
    root = ET.parse(env_sdf).getroot()
    model = root.find(".//model[@name='patrol_environment']")
    if model is None:
        raise RuntimeError("Could not find model patrol_environment in environment SDF")

    env_x = env_y = env_yaw = 0.0
    pose_el = model.find("./pose")
    if pose_el is not None and pose_el.text:
        env_x, env_y, env_yaw = _parse_pose(pose_el.text)

    walls: list[tuple[str, float, float, float, float, float]] = []
    for link in model.findall("./link"):
        name = link.attrib.get("name", "")
        if not name.startswith("Wall_"):
            continue
        pose_el = link.find("./pose")
        size_el = link.find("./collision/geometry/box/size")
        if pose_el is None or not pose_el.text or size_el is None or not size_el.text:
            continue
        lx, ly, yaw = _parse_pose(pose_el.text)
        sx, sy, *_ = [float(v) for v in size_el.text.strip().split()]
        walls.append((name, env_x + lx, env_y + ly, env_yaw + yaw, sx / 2.0, sy / 2.0))

    if not walls:
        raise RuntimeError("No Wall_* links found in environment SDF")
    return walls


def _load_obstacles(world_sdf: pathlib.Path) -> list[tuple[str, float, float, float, float, float]]:
    root = ET.parse(world_sdf).getroot()
    world = root.find(".//world")
    if world is None:
        raise RuntimeError("Could not find <world> in world SDF")

    obstacles: list[tuple[str, float, float, float, float, float]] = []
    for model in world.findall("./model"):
        name = model.attrib.get("name", "")
        if not name.startswith("obstacle_"):
            continue
        pose_el = model.find("./pose")
        size_el = model.find(".//link/collision/geometry/box/size")
        if pose_el is None or not pose_el.text or size_el is None or not size_el.text:
            continue
        x, y, yaw = _parse_pose(pose_el.text)
        sx, sy, _sz = [float(v) for v in size_el.text.strip().split()]
        obstacles.append((name, x, y, yaw, sx, sy))
    obstacles.sort(key=lambda t: t[0])
    return obstacles


def _robot_width_estimate(robot_sdf: pathlib.Path) -> float:
    root = ET.parse(robot_sdf).getroot()
    model = root.find(".//model[@name='patrol_robot']")
    if model is None:
        raise RuntimeError("Could not find model patrol_robot in robot SDF")

    base_size_el = model.find("./link[@name='base_link']/collision/geometry/box/size")
    if base_size_el is None or not base_size_el.text:
        raise RuntimeError("Could not find base_link box size")
    _sx, sy, _sz = [float(v) for v in base_size_el.text.strip().split()]
    base_half = sy / 2.0

    wheel_len_el = model.find("./link[@name='left_wheel_link']/collision/geometry/cylinder/length")
    if wheel_len_el is None or not wheel_len_el.text:
        raise RuntimeError("Could not find left_wheel_link cylinder length")
    wheel_half = float(wheel_len_el.text.strip()) / 2.0

    left_pose_el = model.find("./link[@name='left_wheel_link']/pose")
    right_pose_el = model.find("./link[@name='right_wheel_link']/pose")
    if left_pose_el is None or not left_pose_el.text or right_pose_el is None or not right_pose_el.text:
        raise RuntimeError("Could not find wheel link pose(s)")
    ly = float(left_pose_el.text.strip().split()[1])
    ry = float(right_pose_el.text.strip().split()[1])
    half_width = max(base_half, abs(ly) + wheel_half, abs(ry) + wheel_half)
    return 2.0 * half_width


def _min_wall_clearance(
    walls: list[tuple[str, float, float, float, float, float]],
    name: str,
    x: float,
    y: float,
    yaw: float,
    sx: float,
    sy: float,
) -> tuple[float, str, float | None]:
    obs_poly = _rect_vertices(x, y, yaw, sx / 2.0, sy / 2.0)

    distances: list[tuple[float, str, float, float]] = []
    for wall_name, wx, wy, wyaw, whx, why in walls:
        wall_poly = _rect_vertices(wx, wy, wyaw, whx, why)
        distances.append((_polygons_distance(obs_poly, wall_poly), wall_name, wx, wy))

    distances.sort(key=lambda t: t[0])
    min_d, min_wall, min_wx, min_wy = distances[0]
    ref_v = (min_wx - x, min_wy - y)
    ref_norm2 = ref_v[0] * ref_v[0] + ref_v[1] * ref_v[1]
    if ref_norm2 <= 1e-9:
        return min_d, min_wall, None

    opp_min: float | None = None
    for d, _wall_name, wx, wy in distances[1:]:
        v = (wx - x, wy - y)
        if _dot(v, ref_v) < 0.0:
            opp_min = d
            break
    return min_d, min_wall, opp_min


def main() -> int:
    parser = argparse.ArgumentParser(description="Check obstacle-to-wall clearances vs robot width.")
    parser.add_argument("--robot", default="models/patrol_robot/model.sdf", help="Path to robot model.sdf")
    parser.add_argument(
        "--environment", default="models/patrol_environment/model.sdf", help="Path to environment model.sdf"
    )
    parser.add_argument("--world", default="worlds/patrol_world.sdf", help="Path to world SDF")
    parser.add_argument(
        "--margin",
        type=float,
        default=0.10,
        help="Recommended extra clearance beyond robot width (meters)",
    )
    args = parser.parse_args()

    robot_sdf = pathlib.Path(args.robot)
    env_sdf = pathlib.Path(args.environment)
    world_sdf = pathlib.Path(args.world)

    robot_width = _robot_width_estimate(robot_sdf)
    required_gap = robot_width + max(0.0, float(args.margin))
    walls = _load_walls(env_sdf)
    obstacles = _load_obstacles(world_sdf)

    print(f"Robot width estimate: {robot_width:.3f} m")
    print(f"Recommended min gap: {required_gap:.3f} m (width + margin={args.margin:.2f})")
    print("")
    print("Obstacle clearances to nearest wall:")
    for name, x, y, yaw, sx, sy in obstacles:
        d_near, wall_name, d_opp = _min_wall_clearance(walls, name, x, y, yaw, sx, sy)
        ok = "OK" if d_near >= required_gap else "TIGHT"
        opp_str = "-" if d_opp is None else f"{d_opp:.3f}"
        print(
            f"- {name}: size={sx:.2f}x{sy:.2f} pose=({x:.3f},{y:.3f}) "
            f"near={d_near:.3f}m ({ok}) opp≈{opp_str}m"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

