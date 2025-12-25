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
mkdir -p ~/patrol_ws/src
cd ~/patrol_ws/src
git clone https://github.com/ouyyyyang/robot_work.git

cd ~/patrol_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 方式 B：把本仓库当作工作空间根目录（不推荐但可用）

如果你就是在 `~/Desktop/robot_work`（本仓库根目录）下开发，也可以直接在这里构建，但需要显式告诉 colcon 去哪些目录找包：

```bash
cd ~/Desktop/robot_work
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths patrol_bringup patrol_control
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
cd ~/patrol_ws   # 或 cd ~/Desktop/robot_work（取决于你用哪种方式构建）
source install/setup.bash
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
- 超声：`/patrol_robot/ultrasonic/range`（由 `ultrasonic/scan` 转换得到）
- 超声原始射线：`/patrol_robot/ultrasonic/scan`（`sensor_msgs/LaserScan`，单束）
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
