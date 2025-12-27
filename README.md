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

启动整套系统（Gazebo + 控制节点，Nav2 默认关闭）：

```bash
cd ~/patrol_ws   # 或 cd ~/Desktop/robot_work（取决于你用哪种方式构建）
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch patrol_bringup patrol.launch.py
```

调快巡逻速度 / 提前避障（示例）：

```bash
ros2 launch patrol_bringup patrol.launch.py \
  max_linear:=0.6 \
  obstacle_stop_distance:=0.7
```

巡检点停留时间（默认 2 秒）：

```bash
ros2 launch patrol_bringup patrol.launch.py dwell_time:=2.0
```

巡逻循环（默认开启，跑完 1~5 后回到 1）：

```bash
ros2 launch patrol_bringup patrol.launch.py loop_patrol:=true
```

可选启用 Nav2（需要你补齐定位/地图/TF 等配置）：

```bash
ros2 launch patrol_bringup patrol.launch.py use_nav2:=true
```

### 方式 B：直接打开 Gazebo world（不含自动生成机器人）

```bash
# 在仓库根目录（包含 models/ 和 worlds/）
cd ~/Desktop/robot_work
export GAZEBO_MODEL_PATH="$PWD/models:${GAZEBO_MODEL_PATH}"
gazebo --verbose worlds/patrol_world.sdf
```

## 关键话题

- 机器人速度：`/patrol_robot/cmd_vel`
- 里程计：`/patrol_robot/odom`
- 相机：`/patrol_robot/front_camera/image_raw`
- 超声：`/patrol_robot/ultrasonic/range`（由 `ultrasonic/scan` 转换得到）
- 超声原始射线：`/patrol_robot/ultrasonic/scan`（`sensor_msgs/LaserScan`，61 束，约 πrad 视场角（车头前方半圆））
- 视觉判别结果：`/patrol/vision/status`（`normal`=蓝色，`abnormal`=红色）

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

- `Fixed Frame` 设为 `odom`
- `Global Options -> Use Sim Time` 设为 `true`（否则 TF 会因为时间戳不匹配而显示不出来）
- Add → `TF`（看 TF 树）
- Add → `Odometry`，Topic 选 `/patrol_robot/odom`
- Add → `LaserScan`，Topic 选 `/patrol_robot/ultrasonic/scan`
- Add → `Image`，Topic 选 `/patrol_robot/front_camera/image_raw`
- Add → `RobotModel`（显示机器人 3D 模型，使用 `robot_description`）
- Add → `MarkerArray`，Topic 选 `/patrol/markers`（显示墙体/巡检点/障碍物）

说明：`gazebo.launch.py` 会启动 `robot_state_publisher`（发布机器人各 link 的 TF）和 `/joint_states`（轮子随运动转动），所以 RViz2 可以看到完整机器人模型与传感器 TF。
提示：如果你把 `Fixed Frame` 设为 `map`，但没有启动 Nav2/SLAM（没有 `map->odom` TF），RViz2 会显示不出来，这是正常的。

### 看“雷达/超声”可视化

- Gazebo：`models/patrol_robot/model.sdf` 里 `ultrasonic_sensor` 已设置 `<visualize>true</visualize>`，重启仿真后能在 Gazebo 里看到扇形射线。
- RViz2：Add → `LaserScan` 订阅 `/patrol_robot/ultrasonic/scan` 就能看到扫描扇形；`/patrol_robot/ultrasonic/range` 是 `sensor_msgs/Range`（单个距离值），更适合 `rqt_plot`/`ros2 topic echo` 看数值。

## 巡逻路径规划（基于墙体）

`patrol_control/patrol_manager` 默认会解析 `models/patrol_environment/model.sdf` 里的 `Wall_*` 碰撞盒，构建 2D 栅格并在赛道区域里做 A* 规划，避免“直线去目标点会穿墙”的问题。

常用启动参数（`patrol.launch.py`）：

```bash
# 关闭路径规划（退回为直线点到点 + 超声避障）
ros2 launch patrol_bringup patrol.launch.py use_path_planner:=false

# 调精度/安全边距
ros2 launch patrol_bringup patrol.launch.py grid_resolution:=0.05 wall_inflation:=0.30
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
  -p point_a_x:=3.974 -p point_a_y:=5.786 \
  -p point_b_x:=4.024 -p point_b_y:=3.836 \
  -p speed:=0.4
```

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
