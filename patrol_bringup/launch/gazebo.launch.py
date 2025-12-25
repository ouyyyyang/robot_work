import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("patrol_bringup")
    gazebo_ros_share = FindPackageShare("gazebo_ros")

    world_default = PathJoinSubstitution([bringup_share, "worlds", "patrol_world.sdf"])
    models_dir = PathJoinSubstitution([bringup_share, "models"])
    robot_sdf = PathJoinSubstitution([bringup_share, "models", "patrol_robot", "model.sdf"])

    world_arg = DeclareLaunchArgument("world", default_value=world_default)
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    robot_x_arg = DeclareLaunchArgument("robot_x", default_value="1.482")
    robot_y_arg = DeclareLaunchArgument("robot_y", default_value="5.779")
    robot_z_arg = DeclareLaunchArgument("robot_z", default_value="0.0")
    robot_yaw_arg = DeclareLaunchArgument("robot_yaw", default_value="-0.965")

    set_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            models_dir,
            TextSubstitution(text=":"),
            EnvironmentVariable("GAZEBO_MODEL_PATH"),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "verbose": "true",
        }.items(),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "patrol_robot",
            "-file",
            robot_sdf,
            "-x",
            LaunchConfiguration("robot_x"),
            "-y",
            LaunchConfiguration("robot_y"),
            "-z",
            LaunchConfiguration("robot_z"),
            "-Y",
            LaunchConfiguration("robot_yaw"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            use_sim_time_arg,
            robot_x_arg,
            robot_y_arg,
            robot_z_arg,
            robot_yaw_arg,
            set_model_path,
            gazebo,
            spawn_robot,
        ]
    )
