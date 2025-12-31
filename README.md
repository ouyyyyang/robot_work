# Patrol Robot (ROS 2 Humble + Gazebo Classic)

一个 ROS 2 Humble + Gazebo Classic 的巡逻机器人仿真示例：巡逻环境（墙/巡检点/障碍物）、三轮差速机器人（相机 + 超声），以及巡逻/避障/视觉判别节点。

## 目录结构

- `patrol_bringup/`：启动与仿真资源打包（launch/world/models/URDF）
- `patrol_control/`：控制节点（巡逻、动态障碍、视觉判别等）
- `models/`：Gazebo 模型源文件（环境、机器人）
- `worlds/`：Gazebo world 源文件
- `tools/`：可选脚本
- `PROJECT.md`：实现细节/设计说明

## 环境依赖

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo Classic 11（`gazebo`）+ `gazebo_ros_pkgs`
- `colcon`：`python3-colcon-common-extensions`
- Python3 + `numpy`

Ubuntu 安装示例：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-numpy \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins
```

## 构建（推荐：标准 ROS 2 工作空间）

```bash
mkdir -p ~/patrol_ws/src
cd ~/patrol_ws/src
git clone https://github.com/ouyyyyang/robot_work.git

cd ~/patrol_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 运行（主要入口）

```bash
cd ~/patrol_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch patrol_bringup patrol.launch.py
```

可选用法：

- 只启动 Gazebo + 生成机器人（不启用自动巡逻）：`ros2 launch patrol_bringup gazebo.launch.py`
- 关闭自动巡逻（便于手动测试）：`ros2 launch patrol_bringup patrol.launch.py enable_patrol:=false`

### 手动键盘控制（可选）

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/patrol_robot/cmd_vel
```

若缺少包：`sudo apt install -y ros-humble-teleop-twist-keyboard`

## 常用话题

- `/patrol_robot/cmd_vel`：速度指令
- `/patrol_robot/odom`：里程计
- `/patrol_robot/front_camera/image_raw`：相机图像
- `/patrol_robot/ultrasonic/range`：超声距离
- `/patrol/vision/status`：巡检点状态（`normal`/`abnormal`）
- `/patrol/markers`：环境 MarkerArray

## 文档

- 设计/实现细节：`PROJECT.md`
- 可配置的 launch 参数：`patrol_bringup/launch/patrol.launch.py`

## License

Apache-2.0，见 `LICENSE`。
