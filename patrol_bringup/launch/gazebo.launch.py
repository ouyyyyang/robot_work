from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


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

    urdf_path = get_package_share_directory("patrol_bringup") + "/urdf/patrol_robot.urdf"
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "robot_description": robot_description,
            }
        ],
    )

    wheel_joint_state_publisher = Node(
        package="patrol_control",
        executable="wheel_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            }
        ],
    )

    odom_tf_broadcaster = Node(
        package="patrol_control",
        executable="odom_tf_broadcaster",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            }
        ],
    )

    environment_markers = Node(
        package="patrol_control",
        executable="environment_markers",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            }
        ],
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
            robot_state_publisher,
            wheel_joint_state_publisher,
            odom_tf_broadcaster,
            environment_markers,
        ]
    )
