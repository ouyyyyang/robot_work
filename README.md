# Patrol Robot (ROS2 Humble + Gazebo Classic)

本工程包含巡逻环境（墙/巡检点/障碍物）、三轮巡逻机器人模型（两驱动轮+万向轮+单目相机+超声）、以及对应的 ROS2 启动与控制节点示例。

## 目录结构

- `models/`：Gazebo 模型目录（建议只放模型，方便设置 `GAZEBO_MODEL_PATH`）
  - `models/patrol_environment/`：墙体 + 巡检点（红/蓝状态）
  - `models/patrol_robot/`：三轮机器人（diff-drive）+ 相机 + 超声
- `worlds/patrol_world.sdf`：世界文件（地面 + 环境模型 + 5 个障碍物）
- `patrol_bringup/`：ROS2 启动包（Gazebo + 可选 Nav2）
- `patrol_control/`：ROS2 控制节点（巡检/动态障碍/视觉判别）

## 依赖

- ROS2 Humble
- Gazebo Classic (gazebo11) + `gazebo_ros_pkgs`
- Python3 + `rclpy`
- `python3-numpy`（`vision_checker` 使用）

## 构建

在你的 ROS2 工作空间里（或把本工程作为 workspace 根目录）：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 运行

### 方式 A：ROS2 启动（推荐）

启动 Gazebo 并生成机器人：

```bash
ros2 launch patrol_bringup gazebo.launch.py
```

启动整套系统（Gazebo + 控制节点，Nav2 默认关闭）：

```bash
ros2 launch patrol_bringup patrol.launch.py
```

可选启用 Nav2（需要你补齐定位/地图/TF 等配置）：

```bash
ros2 launch patrol_bringup patrol.launch.py use_nav2:=true
```

### 方式 B：直接打开 Gazebo world（不含自动生成机器人）

```bash
export GAZEBO_MODEL_PATH="$PWD/models:${GAZEBO_MODEL_PATH}"
gazebo --verbose worlds/patrol_world.sdf
```

## 关键话题

- 机器人速度：`/patrol_robot/cmd_vel`
- 里程计：`/patrol_robot/odom`
- 相机：`/patrol_robot/front_camera/image_raw`
- 超声：`/patrol_robot/ultrasonic/range`
- 视觉判别结果：`/patrol/vision/status`（`normal`=蓝色，`abnormal`=红色）

## 动态障碍（两点往返）

`patrol_control/obstacle_controller` 会把指定障碍物在两点之间循环往返移动（通过 `/gazebo/set_entity_state`）。

常用参数：

- `obstacles`：需要移动的障碍物名列表（例如 `['obstacle_1']`）
- `point_a_x/point_a_y`、`point_b_x/point_b_y`：两个端点坐标（world 坐标系）
- `speed`：移动速度（m/s）

建议：把 `worlds/patrol_world.sdf` 里对应障碍物的初始 pose 放在 A 点附近，避免节点启动后第一次更新造成“瞬移”。

示例（只移动 `obstacle_1`，在两点间往返）：

```bash
ros2 run patrol_control obstacle_controller --ros-args \
  -p obstacles:="[obstacle_1]" \
  -p point_a_x:=-3.0 -p point_a_y:=0.9 \
  -p point_b_x:=-1.0 -p point_b_y:=0.9 \
  -p speed:=0.4
```

## 巡检点状态（红/蓝）

巡检点颜色在 `models/patrol_environment/model.sdf` 中设置：

- 蓝色：`normal`
- 红色：`abnormal`
