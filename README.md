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
- `colcon`（需要带 ROS 扩展：`python3-colcon-common-extensions` / `python3-colcon-ros`）
- Python3 + `rclpy`
- `python3-numpy`（`vision_checker` 使用）

## 快速开始（推荐流程）

下面以 Ubuntu + ROS2 Humble 为例，按步骤从 0 开始跑起来（每一步都标注了运行目录）。

### 0) 准备工作（只需要做一次）

```bash
# 任意目录
source /opt/ros/humble/setup.bash
```

## 构建

你需要在“工作空间根目录”（会生成 `build/ install/ log/` 的地方）运行 `colcon build`，不要在 `src/` 里运行。

### 方式 A：标准 ROS2 工作空间（推荐）

目录结构示例：

```
~/patrol_ws/
  src/
    robot_work/   (本仓库)
```

命令（注意 `cd` 的路径）：

```bash
# 1) 创建工作空间
mkdir -p ~/patrol_ws/src
cd ~/patrol_ws/src

# 2) 拉取代码（把本仓库放到 src 下）
git clone https://github.com/ouyyyyang/robot_work.git

# 3) 回到工作空间根目录构建（这里会生成 build/ install/ log/）
cd ~/patrol_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 方式 B：把本仓库当作工作空间根目录（不推荐但可用）

如果你就是在 `~/Desktop/robot_work`（本仓库根目录）下开发，也可以直接在这里构建，但需要显式告诉 colcon 去哪些目录找包：

```bash
# 在仓库根目录构建
cd ~/Desktop/robot_work
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths patrol_bringup patrol_control
source install/setup.bash
```

## 运行

### 方式 A：ROS2 启动（推荐）

启动 Gazebo 并生成机器人：

```bash
# 先加载 overlay（新开一个终端也需要重新 source）
cd ~/patrol_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch patrol_bringup gazebo.launch.py
```

说明：`gazebo.launch.py` 只负责启动 Gazebo + 生成机器人，不会让车自动巡逻。

#### 键盘控制（手动测试移动）

建议只启动 `gazebo.launch.py`（不要启动 `patrol.launch.py`），避免 `patrol_manager` 同时发布 `/patrol_robot/cmd_vel` 和键盘抢控制。

另开一个终端执行（把键盘速度指令重映射到 `/patrol_robot/cmd_vel`）：

```bash
cd ~/patrol_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/patrol_robot/cmd_vel
```

如果提示 `Package 'teleop_twist_keyboard' not found`，安装：

```bash
sudo apt update
sudo apt install -y ros-humble-teleop-twist-keyboard
```

常用按键（以终端提示为准）：

- 直走/后退：`i` 前进、`,` 后退、`k` 停止
- 原地转向：`j` 左转、`l` 右转
- 边走边转：`u` 前进左转、`o` 前进右转、`m` 后退左转、`.` 后退右转

说明：终端里无法“同时按住两键组合”（比如按着 `i` 再按 `j`），每次按键都会直接发送一条新的 `/cmd_vel`，所以要边走边转请用 `u/o/m/.` 这类组合键。

启动整套系统（Gazebo + 控制节点）：

```bash
cd ~/patrol_ws   # 或 cd ~/Desktop/robot_work（取决于你用哪种方式构建）
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch patrol_bringup patrol.launch.py
```

默认 `use_nav2:=false`：使用 `patrol_control/patrol_manager`（自研巡逻 + 超声避障 + 可选 A* 规划）。

启用 `use_nav2:=true`：使用 **Nav2 + slam_toolbox** 进行建图与导航，并切换为 `patrol_control/nav2_patrol_manager` 依次巡检 1~5 号检查点。

调快巡逻速度 / 提前避障（示例）：

```bash
ros2 launch patrol_bringup patrol.launch.py \
  max_linear:=0.6 \
  obstacle_stop_distance:=0.7
```

如果避障时出现“抽搐/来回切换”，可以增大退出避障的阈值和保持时间（示例）：

```bash
ros2 launch patrol_bringup patrol.launch.py \
  obstacle_clear_distance:=0.8 \
  avoid_min_turn_time:=0.5 \
  avoid_clear_hold_time:=0.25 \
  avoid_dir_hysteresis:=0.12
```

如果在墙角转弯容易“顶住角”或靠近障碍物左右摆头，可以调大侧向扫描范围并增加扫描引导的平滑（示例）：

```bash
ros2 launch patrol_bringup patrol.launch.py \
  scan_side_angle:=1.57 \
  goal_min_weight:=0.2 \
  goal_turn_angle:=1.2 \
  scan_forward_bias:=0.35 \
  scan_angle_smoothing:=0.35 \
  scan_angle_deadband:=0.08 \
  scan_goal_align_weight:=0.8 \
  scan_prev_align_weight:=0.5
```

如果出现“转弯转过头/方向反了”的情况，可以让大角速度时原地转向（示例）：

```bash
ros2 launch patrol_bringup patrol.launch.py turn_in_place_angular:=1.0
```

巡检点停留时间（默认 2 秒）：

```bash
ros2 launch patrol_bringup patrol.launch.py dwell_time:=2.0
```

巡逻循环（默认开启，跑完 1~5 后回到 1）：

```bash
ros2 launch patrol_bringup patrol.launch.py loop_patrol:=true
```

启用 Nav2 + SLAM（推荐）

先安装依赖（只需要做一次）：

```bash
sudo apt update
sudo apt install -y ros-humble-nav2-* ros-humble-slam-toolbox
```

启动（默认 `slam:=true`）：

```bash
ros2 launch patrol_bringup patrol.launch.py use_nav2:=true
```

如果 Gazebo GUI 不弹窗（虚拟机/显卡驱动常见），可以无 GUI 运行：

```bash
ros2 launch patrol_bringup patrol.launch.py use_nav2:=true gui:=false
```

（仍不行再尝试）强制软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch patrol_bringup patrol.launch.py use_nav2:=true gui:=false
```

如果你已经保存了地图，想用“定位+导航”（不建图），可以这样启动：

```bash
ros2 launch patrol_bringup patrol.launch.py use_nav2:=true slam:=false map:=/absolute/path/to/your_map.yaml
```

巡检结果会在到达每个检查点并停留后发布到：

- `/patrol/inspection/report`（格式：`patrol_point_N:normal|abnormal`）

### 方式 B：直接打开 Gazebo world（不含自动生成机器人）

```bash
# 在仓库根目录（包含 models/ 和 worlds/）
cd ~/Desktop/robot_work
export GAZEBO_MODEL_PATH="$PWD/models:${GAZEBO_MODEL_PATH}"
gazebo --verbose worlds/patrol_world.sdf
```

## 关键话题

- 机器人速度：`/patrol_robot/cmd_vel`
- Nav2 速度（转发前）：`/cmd_vel`、`/cmd_vel_smoothed`、`/cmd_vel_nav`（由 `patrol_control/twist_relay` 转发到 `/patrol_robot/cmd_vel`）
- 里程计：`/patrol_robot/odom`
- 地图（Nav2+SLAM）：`/map`
- 相机：`/patrol_robot/front_camera/image_raw`
- 超声：`/patrol_robot/ultrasonic/range`（由 `ultrasonic/scan` 转换得到）
- 超声原始射线：`/patrol_robot/ultrasonic/scan`（`sensor_msgs/LaserScan`，61 束，约 πrad 视场角（车头前方半圆））
- 视觉判别结果：`/patrol/vision/status`（`normal`=蓝色，`abnormal`=红色；只有当相机 ROI 内红/蓝像素占比超过阈值时才发布，避免把黄色障碍等其它颜色误判为检查点）
- 环境标记：`/patrol/markers`（墙体/巡检点/障碍物 MarkerArray）
- 巡检报告（Nav2+SLAM）：`/patrol/inspection/report`

## RViz2 可视化

先启动仿真（例如）：

```bash
cd ~/patrol_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch patrol_bringup patrol.launch.py
```

再开 RViz2：

```bash
cd ~/patrol_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

RViz2 里建议这样配置：

- `Fixed Frame`：
  - 不用 Nav2：设为 `odom`
  - 用 Nav2+SLAM：设为 `map`
- `Global Options -> Use Sim Time` 设为 `true`（否则 TF 会因为时间戳不匹配而显示不出来）
- Add → `TF`（看 TF 树）
- Add → `Odometry`，Topic 选 `/patrol_robot/odom`
- Add → `LaserScan`，Topic 选 `/patrol_robot/ultrasonic/scan`
- Add → `Image`，Topic 选 `/patrol_robot/front_camera/image_raw`
- Add → `RobotModel`（显示机器人 3D 模型，使用 `robot_description`）
- Add → `MarkerArray`，Topic 选 `/patrol/markers`（显示墙体/巡检点/障碍物）
- （Nav2+SLAM）Add → `Map`，Topic 选 `/map`

说明：`gazebo.launch.py` 会启动 `robot_state_publisher`（发布机器人各 link 的 TF）和 `/joint_states`（轮子随运动转动），所以 RViz2 可以看到完整机器人模型与传感器 TF。
提示：如果你把 `Fixed Frame` 设为 `map`，但没有启动 Nav2/SLAM（没有 `map->odom` TF），RViz2 会显示不出来，这是正常的。

### 看“雷达/超声”可视化

- Gazebo：`models/patrol_robot/model.sdf` 里 `ultrasonic_sensor` 已设置 `<visualize>true</visualize>`，重启仿真后能在 Gazebo 里看到扇形射线。
- RViz2：Add → `LaserScan` 订阅 `/patrol_robot/ultrasonic/scan` 就能看到扫描扇形；`/patrol_robot/ultrasonic/range` 是 `sensor_msgs/Range`（单个距离值），更适合 `rqt_plot`/`ros2 topic echo` 看数值。

## 巡逻路径规划（基于墙体）

本节仅适用于 `use_nav2:=false`（使用自研 `patrol_manager`）。启用 Nav2 后，路径规划 / 避障由 Nav2 的全局/局部代价地图与控制器负责。

`patrol_control/patrol_manager` 默认会解析 `models/patrol_environment/model.sdf` 里的 `Wall_*` 碰撞盒，构建 2D 栅格并在赛道区域里做 A* 规划，避免“直线去目标点会穿墙”的问题。
同时支持把 Gazebo 里的 `obstacle_*` 动态纳入规划（订阅 `/gazebo/model_states`），避免“目标点在障碍物后面时左右摆头/卡住”。

常用启动参数（`patrol.launch.py`）：

```bash
# 关闭路径规划（退回为直线点到点 + 超声避障）
ros2 launch patrol_bringup patrol.launch.py use_path_planner:=false

# 调精度/安全边距
ros2 launch patrol_bringup patrol.launch.py grid_resolution:=0.05 wall_inflation:=0.30

# 把动态障碍也纳入规划（更容易绕过盒子障碍）
ros2 launch patrol_bringup patrol.launch.py use_dynamic_obstacles:=true obstacle_inflation:=0.17
```

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
  -p point_a_x:=3.724 -p point_a_y:=4.986 \
  -p point_b_x:=2.824 -p point_b_y:=5.136 \
  -p speed:=0.4
```

## 障碍物-墙缝隙宽度检查（可选）

如果你想确认“障碍物与墙之间的缝隙”是否能让车穿过，可以在仓库根目录运行：

```bash
python3 tools/check_clearances.py --margin 0.10
```

它会根据 `models/patrol_robot/model.sdf` 估算车宽，并计算 `worlds/patrol_world.sdf` 里每个障碍物到最近墙体的最小间隙，方便你判断是否需要移动障碍物/减小尺寸/调整规划参数。

## 巡检点状态（红/蓝）

巡检点颜色在 `models/patrol_environment/model.sdf` 中设置：

- 蓝色：`normal`
- 红色：`abnormal`

## 常见问题

### 1) `ros2 launch` 提示 `package 'patrol_bringup' not found`

这表示当前终端没有加载到你的工作空间 overlay（`~/patrol_ws/install`）。按顺序执行：

```bash
cd ~/patrol_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | head -n 10
ros2 pkg list | grep patrol
```

如果仍然找不到包，先确认 colcon 把它识别成 ROS2 的 `ament_python` 包（正常应该显示 `ament_python`，而不是 `python`）：

```bash
cd ~/patrol_ws
colcon list --base-paths src | grep patrol
```

如果你看到的是 `(python)`：

- 先确认 colcon 的 ROS 扩展存在：

```bash
python3 -c "import colcon_ros; print('colcon_ros ok')"
```

- 如果扩展不存在，在 Ubuntu 上安装并重新打开终端：

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions
```

- 如果扩展存在但仍是 `(python)`，通常是 `package.xml` 不符合 ROS2 规范导致解析失败（例如 `maintainer` 的 `email` 为空）。可用下面命令检查：

```bash
python3 -c "from catkin_pkg.package import parse_package; parse_package('src/robot_work/patrol_bringup/package.xml'); print('patrol_bringup ok')"
python3 -c "from catkin_pkg.package import parse_package; parse_package('src/robot_work/patrol_control/package.xml'); print('patrol_control ok')"
```

然后清理重建：

```bash
cd ~/patrol_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --merge-install
source install/setup.bash
ros2 pkg list | grep patrol
```

如果 `AMENT_PREFIX_PATH` 仍然只有 `/opt/ros/humble`，再试：

```bash
source install/local_setup.bash
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | head -n 10
ros2 pkg list | grep patrol
```

仍不行的话，建议用合并安装重新构建（避免 isolated-install 链接环境失败）：

```bash
cd ~/patrol_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --merge-install
source install/setup.bash
ros2 pkg list | grep patrol
```

如果你的 shell 是 zsh，请改用：

```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```

### 2) Gazebo 报 `Failed to load plugin libgazebo_ros_*.so`

例如：

- `libgazebo_ros_diff_drive.so`
- `libgazebo_ros_camera.so`
- `libgazebo_ros_ray_sensor.so`

这表示 Gazebo 找不到对应的 ROS-Gazebo 插件库（通常是没有装 `gazebo_plugins`）。

在 Ubuntu (Humble) 上安装：

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

验证插件文件是否存在：

```bash
ls /opt/ros/humble/lib | grep -E "libgazebo_ros_(diff_drive|camera|range)\\.so"
```

### 3) Nav2 一直卡在 `Waiting for service controller_server/get_state...`

这通常表示 Nav2 的 `controller_server` 没有成功启动（最常见原因：缺少控制器插件包 / 参数文件配置错误导致进程直接退出），所以生命周期管理器一直等不到它的 `get_state` 服务。

按下面顺序排查：

```bash
# 1) 看 controller_server 是否存在（正常应能看到）
ros2 node list | grep controller_server

# 2) 看 Nav2 组件容器 / controller_server 的报错日志（通常会提示缺哪个 plugin）
ls -t ~/.ros/log/* | head
```

确保依赖已安装（推荐直接装全套）：

```bash
sudo apt update
sudo apt install -y ros-humble-nav2-* ros-humble-slam-toolbox
```

### 4) Nav2 都启动了，但机器人完全不动

优先按这个顺序排查（通常是 `/cmd_vel` 没有进到 `/patrol_robot/cmd_vel`，或巡检节点没成功发 goal）：

```bash
# 1) 确认巡检 goal 发送节点在跑（use_nav2:=true 时应存在）
ros2 node list | grep nav2_patrol_manager

# 2) 确认 Nav2 的动作服务器存在
ros2 action list | grep -E "^/navigate_to_pose$"

# 3) 看 Nav2 是否在产生速度（这里任意一个有频率都算）
ros2 topic hz /cmd_vel
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel_nav

# 4) 看最终机器人是否收到速度
ros2 topic hz /patrol_robot/cmd_vel

# 5) 确认转发节点在跑（use_nav2:=true 时应存在）
ros2 node list | grep twist_relay
```

### 5) `local_costmap` 报 “Timed out waiting for transform … base_link to odom”

这表示 **TF 里缺少 `odom -> base_link`**（Nav2 必需的里程计坐标变换）。

本工程默认由 Gazebo 的 `libgazebo_ros_diff_drive` 插件发布该 TF（`models/patrol_robot/model.sdf` 里 `publish_odom_tf=true`）。

```bash
ros2 topic echo /patrol_robot/odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

如果 `/patrol_robot/odom` 有数据但还是没有 TF，可以临时用兜底广播器：

```bash
ros2 run patrol_control odom_tf_broadcaster --ros-args -p odom_topic:=/patrol_robot/odom
```

### 6) SLAM 报 “Message Filter dropping message … queue is full”

这通常表示 `slam_toolbox` 等不到对应时间戳的 TF（或扫描频率太高导致积压）。本工程已把超声 `LaserScan` 更新频率设为 10Hz 以适配 `slam_toolbox` 默认处理频率；如果你手动把传感器更新频率调得很高，可能会出现该提示。

你也可以用这些命令确认 TF 与 scan 是否匹配：

```bash
ros2 topic hz /patrol_robot/ultrasonic/scan
ros2 run tf2_ros tf2_echo base_link ultrasonic_link
```
