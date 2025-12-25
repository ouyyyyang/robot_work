# 项目说明（ROS2 Humble + Gazebo Classic）

本项目实现了一个“巡逻机器人仿真系统”，包含：

- **Gazebo world**：地面 + 环境模型 + 5 个可碰撞障碍物（支持动态移动）
- **Gazebo 机器人模型（SDF）**：三轮结构（两驱动轮 + 1 万向轮）+ 单目相机 + 超声波测距
- **ROS2 节点**：巡检点顺序巡逻、动态障碍控制、视觉判别（蓝=正常 / 红=异常）

说明中仅描述机器人与 ROS2 侧实现细节（赛道墙体不展开）。

---

## 1. 仿真组成与关键文件

- 世界文件：`worlds/patrol_world.sdf`
  - `ground`：地面模型（50m × 50m 平面）
  - `<include model://patrol_environment>`：加载巡逻环境（含巡检点）
  - `obstacle_1..5`：5 个盒子障碍物（可碰撞、可被 ROS2 控制移动）
- 环境模型：`models/patrol_environment/model.sdf`
  - `patrol_point_1..5`：巡检点（纯视觉球体标记，蓝/红）
- 机器人模型：`models/patrol_robot/model.sdf`
  - `base_link` + `left/right wheel` + `caster` + `camera` + `ultrasonic`

---

## 2. 机器人模型（尺寸/结构/参数）

机器人是一个简化的三轮差速小车。

### 2.1 底盘（base_link）

文件：`models/patrol_robot/model.sdf`

- 形状：长方体 box
- 尺寸（长×宽×高，单位 m）：**0.45 × 0.30 × 0.10**
- 质量：8.0 kg
- `base_link` 的局部位姿：`(x,y,z)=(0,0,0.1)`（底盘中心离地 0.1m）

### 2.2 左/右驱动轮（差速轮）

- 轮子形状：圆柱 cylinder
- 半径：0.05 m
- 宽度（length）：0.02 m
- 左轮中心位姿：`(0, +0.16, 0.05)`
- 右轮中心位姿：`(0, -0.16, 0.05)`
- 轮子关节：
  - `left_wheel_joint`：revolute（轴向 `xyz=0 1 0`）
  - `right_wheel_joint`：revolute（轴向 `xyz=0 1 0`）

### 2.3 万向轮（caster）

- 形状：球体 sphere
- 半径：0.04 m
- 中心位姿：`(-0.27, 0, 0.04)`
- 关节：`caster_wheel_joint`（ball）

---

## 3. 传感器（单目相机 + 超声）

### 3.1 单目相机

定义位置：

- Link：`camera_link`
- 相对 `base_link` 位姿：`(0.20, 0, 0.18)`
- 相机模型：`<sensor type="camera">`

相机参数：

- 分辨率：640 × 480
- 格式：`R8G8B8`
- 水平视场角（horizontal_fov）：1.047 rad（约 60°）
- 近/远裁剪：0.1 m / 10.0 m
- 更新频率：30 Hz

Gazebo-ROS 插件：

- 插件：`libgazebo_ros_camera.so`
- 命名空间：`/patrol_robot`
- 发布话题：
  - `/patrol_robot/front_camera/image_raw`（`sensor_msgs/Image`）
  - `/patrol_robot/front_camera/camera_info`（`sensor_msgs/CameraInfo`）
- TF frame：`camera_link`

### 3.2 超声波测距（单束 ray）

定义位置：

- Link：`ultrasonic_link`
- 相对 `base_link` 位姿：`(0.23, 0, 0.10)`
- 传感器类型：`<sensor type="ray">`（1 条射线，等价“单点超声距离”）

测距参数：

- `min`：0.02 m
- `max`：4.0 m
- `resolution`：0.01 m
- 更新频率：20 Hz

Gazebo-ROS 插件：

- 插件：`libgazebo_ros_ray_sensor.so`（发布 `LaserScan`，单束）
- 命名空间：`/patrol_robot`
- 发布话题：
  - `/patrol_robot/ultrasonic/scan`（`sensor_msgs/LaserScan`）
  - `/patrol_robot/ultrasonic/range`（`sensor_msgs/Range`，由 `patrol_control/scan_to_range` 转换得到）
- TF frame：`ultrasonic_link`

---

## 4. 差速驱动与里程计（Gazebo 插件）

插件：`libgazebo_ros_diff_drive.so`（定义在 `models/patrol_robot/model.sdf`）

关键参数：

- 左/右轮关节：`left_wheel_joint` / `right_wheel_joint`
- `wheel_separation`：0.32 m
- `wheel_diameter`：0.10 m
- `max_wheel_torque`：20.0
- `max_wheel_acceleration`：10.0
- `odometry_frame`：`odom`
- `robot_base_frame`：`base_link`
- `publish_odom_tf`：true

ROS2 接口（命名空间 `/patrol_robot`）：

- 订阅：
  - `/patrol_robot/cmd_vel`（`geometry_msgs/Twist`）
- 发布：
  - `/patrol_robot/odom`（`nav_msgs/Odometry`）
  - TF：`odom -> base_link`

---

## 5. 巡检点（状态信息：蓝/红）

巡检点在 `models/patrol_environment/model.sdf` 中定义为**可视化球体**（无 collision，不影响运动）。

统一参数：

- 球体半径：0.2 m
- 高度：`z=0.1`（球心高度）

颜色含义：

- **蓝色 = 正常（normal）**
- **红色 = 异常（abnormal）**

默认 5 个点的局部坐标（相对于 `patrol_environment` 模型）：

| 点位 | 局部坐标 (x,y,z) |
|---|---|
| patrol_point_1 | (-6.5, -3.0, 0.1) |
| patrol_point_2 | (-2.0, -5.0, 0.1) |
| patrol_point_3 | ( 3.0, -4.0, 0.1) |
| patrol_point_4 | ( 6.5,  1.5, 0.1) |
| patrol_point_5 | (-1.5,  4.0, 0.1) |

环境模型自身在 world 中的 pose（`models/patrol_environment/model.sdf` 顶部）：

- `patrol_environment.pose = (-2.57827, 0.945924, yaw=0)`

因此在当前 yaw=0 的情况下，巡检点的 world 坐标可近似按：

- `world_x = env_x + local_x`
- `world_y = env_y + local_y`

---

## 6. 障碍物（静态初始 + 可动态移动）

障碍物在 `worlds/patrol_world.sdf` 中定义为 5 个可碰撞盒子：

- 尺寸：0.6 × 0.6 × 0.5（单位 m）
- 初始高度：`z=0.25`（半高）
- 设置：`<kinematic>true</kinematic>`（便于通过 ROS2 服务强制改 pose）

初始 pose（world 坐标）：

| 障碍物 | 初始 pose (x,y,z) |
|---|---|
| obstacle_1 | (-4.0, -4.0, 0.25) |
| obstacle_2 | (-1.0, -2.0, 0.25) |
| obstacle_3 | ( 2.0, -1.5, 0.25) |
| obstacle_4 | ( 4.5,  1.5, 0.25) |
| obstacle_5 | ( 0.0,  3.0, 0.25) |

---

## 7. ROS2 软件架构（节点/话题/服务/动作）

### 7.1 Launch

- `patrol_bringup/launch/gazebo.launch.py`
  - 启动 Gazebo（通过 `gazebo_ros` 的标准 launch）
  - 设置 `GAZEBO_MODEL_PATH` 指向安装后的 `patrol_bringup/models`
  - 通过 `gazebo_ros/spawn_entity.py` 生成机器人
  - 机器人初始位置参数：
    - `robot_x=-2.5`、`robot_y=0.95`、`robot_z=0.0`、`robot_yaw=0.0`

- `patrol_bringup/launch/patrol.launch.py`
  - 包含 `gazebo.launch.py`
  - 启动控制节点：
    - `patrol_control/vision_checker`
    - `patrol_control/obstacle_controller`
    - `patrol_control/patrol_manager`
  - Nav2：提供可选入口（`use_nav2:=true`），但参数文件目前为占位，需要你自行补全

### 7.2 节点：`vision_checker`（视觉判别）

文件：`patrol_control/patrol_control/vision_checker.py`

- 订阅：
  - `/patrol_robot/front_camera/image_raw`（`sensor_msgs/Image`）
- 发布：
  - `/patrol/vision/status`（`std_msgs/String`）：`normal` / `abnormal` / `unknown`

算法（默认参数）：

- 取图像中心 ROI：`roi_size=80`（80×80 像素，最小 10×10）
- 计算 ROI 的 RGB 均值：`mean_r, mean_g, mean_b`
- 判别规则（`dominance_ratio=1.25`）：
  - 若 `mean_r / mean_b >= 1.25` → `abnormal`（红）
  - 若 `mean_b / mean_r >= 1.25` → `normal`（蓝）
  - 否则 → `unknown`

### 7.3 节点：`patrol_manager`（巡检顺序控制 + 简单避障）

文件：`patrol_control/patrol_control/patrol_manager.py`

订阅/发布：

- 订阅：
  - `/patrol_robot/odom`（`nav_msgs/Odometry`）
  - `/patrol_robot/ultrasonic/range`（`sensor_msgs/Range`）
  - `/patrol/vision/status`（`std_msgs/String`）
- 发布：
  - `/patrol_robot/cmd_vel`（`geometry_msgs/Twist`）

巡检点来源：

- 启动时解析 `models/patrol_environment/model.sdf` 内的 `patrol_point_1..5` pose
- 结合 `patrol_environment.pose` 计算得到 world 坐标（目前 yaw=0，等价平移）

运动控制算法（纯几何/P 控制，默认参数）：

- 到目标点距离：`dist = hypot(dx, dy)`
- 目标航向：`target = atan2(dy, dx)`
- 航向误差：`err = normalize(target - yaw)`
- 角速度：`w = clamp(k_angular * err, ±max_angular)`
- 线速度：`v = min(max_linear, k_linear * dist)`
- 当 `|err|` 较大时降低线速度：
  - `|err| > 0.8` → `v *= 0.2`
  - `|err| > 0.4` → `v *= 0.6`

到点与“状态采集”：

- 当 `dist <= goal_tolerance`：
  - 停车，等待 `vision_checker` 发布 `normal/abnormal`
  - 记录（log）该点状态后切换到下一个点

避障算法（反应式，默认参数）：

- 若 `ultrasonic.range < obstacle_stop_distance`：
  - `v=0`
  - `w=avoid_turn_angular`（原地转向避让）

默认关键参数：

- `goal_tolerance=0.35`
- `k_linear=0.6`，`max_linear=0.35`
- `k_angular=1.8`，`max_angular=1.2`
- `obstacle_stop_distance=0.45`
- `avoid_turn_angular=1.0`

### 7.3.1 节点：`scan_to_range`（超声 LaserScan → Range）

文件：`patrol_control/patrol_control/scan_to_range.py`

- 订阅：
  - `/patrol_robot/ultrasonic/scan`（`sensor_msgs/LaserScan`）
- 发布：
  - `/patrol_robot/ultrasonic/range`（`sensor_msgs/Range`）

转换规则（默认）：

- 优先取 `LaserScan.ranges` 的中间束；无效时取所有有限值的最小值；全部无效则取 `range_max`。
- `radiation_type = ULTRASOUND`，`field_of_view` 优先用 `|angle_max-angle_min|`，否则用 `default_fov=0.2`。

### 7.4 节点：`obstacle_controller`（动态障碍两点往返）

文件：`patrol_control/patrol_control/obstacle_controller.py`

功能：

- 通过 Gazebo 服务 **强制修改障碍物 pose**，让障碍物在两点之间往返运动

接口：

- 使用服务：`/gazebo/set_entity_state`（`gazebo_msgs/srv/SetEntityState`）
- 控制对象：参数 `obstacles`（默认 `["obstacle_1"]`）

轨迹算法：

- 给定端点 A(ax,ay) 与 B(bx,by)，线段长度 `L = hypot(bx-ax, by-ay)`
- 维护一个“沿线距离相位” `phase ∈ [0, 2L)`，每次更新：
  - `phase = (phase + speed * dt) mod (2L)`
- 映射到插值因子 `s`：
  - 若 `phase <= L`：`s = phase / L`（A → B）
  - 否则：`s = 2 - phase / L`（B → A）
- 最终位置：
  - `x = ax + s*(bx-ax)`
  - `y = ay + s*(by-ay)`
  - `z` 固定为参数（默认 0.25）

注意：

- 轨迹参数 A/B 使用 **world 坐标系**；障碍物必须在 `worlds/patrol_world.sdf` 中以同名模型存在（例如 `obstacle_1`）。
- `obstacle_controller` 运行后会持续用服务写入 pose；为避免启动后“瞬移”，建议把 `worlds/patrol_world.sdf` 里该障碍物的初始 pose 设在 A 点附近。
- 该节点**不会**做碰撞/边界检查；如果 A-B 线段穿过墙体或其它静态物体，会导致障碍物与静态模型发生重叠/穿模。
- 需要不同障碍物走不同轨迹时，建议启动多个 `obstacle_controller`（每个节点控制不同 `obstacles` 与不同 A/B）。

### 7.5 服务与动作（Actions）

本项目当前用到的 ROS2 **服务**：

- `/gazebo/set_entity_state`：动态障碍移动（`obstacle_controller`）
- `/spawn_entity`（由 `gazebo_ros_factory` 提供）：生成机器人（`spawn_entity.py` 在 launch 中调用）

本项目当前 **未直接使用 ROS2 Action**：

- `patrol_manager` 采用直接发布 `/cmd_vel` 的点到点控制（非 Nav2 action）。
- `patrol.launch.py` 中留有 Nav2 bringup 的入口，但要真正使用 Nav2（如 `NavigateToPose` action），需要你补地图/定位/代价地图等配置。

---

## 8. 快速运行（Humble）

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

启动整套系统：

```bash
ros2 launch patrol_bringup patrol.launch.py
```

动态障碍参数示例（让 `obstacle_1` 在两点间往返）：

```bash
ros2 launch patrol_bringup patrol.launch.py \
  obstacle_a_x:=-3.0 obstacle_a_y:=0.95 \
  obstacle_b_x:=-1.0 obstacle_b_y:=0.95 \
  obstacle_speed:=0.4
```
