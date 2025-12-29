from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import UnlessCondition
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
    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    # If true, rely on `libgazebo_ros_joint_state_publisher.so` in the robot SDF to publish /joint_states.
    # Set to false to use the Python fallback node (useful if the plugin isn't available on your system).
    use_gazebo_joint_states_arg = DeclareLaunchArgument("use_gazebo_joint_states", default_value="true")
    robot_x_arg = DeclareLaunchArgument("robot_x", default_value="1.482")
    robot_y_arg = DeclareLaunchArgument("robot_y", default_value="5.779")
    robot_z_arg = DeclareLaunchArgument("robot_z", default_value="0.0")
    robot_yaw_arg = DeclareLaunchArgument("robot_yaw", default_value="-0.965")

    set_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            models_dir,
            TextSubstitution(text=":/usr/share/gazebo-11/models:"),
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
    )

    # Ensure Gazebo can find ROS-Gazebo plugins even if the user didn't source ROS env in this shell.
    # Also keep Gazebo's default plugin dirs in case `GAZEBO_PLUGIN_PATH` was unset.
    set_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=[
            TextSubstitution(
                text="/opt/ros/humble/lib:"
                "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:"
                "/usr/lib/aarch64-linux-gnu/gazebo-11/plugins:"
                "/usr/lib/gazebo-11/plugins:"
            ),
            EnvironmentVariable("GAZEBO_PLUGIN_PATH", default_value=""),
        ],
    )

    # Prevent Gazebo from blocking on online model database access (common in restricted networks).
    disable_model_db = SetEnvironmentVariable(name="GAZEBO_MODEL_DATABASE_URI", value=TextSubstitution(text=""))

    # Helpful on some VMs / remote sessions (uncomment if needed):
    #   export LIBGL_ALWAYS_SOFTWARE=1
    # We keep it opt-in via env, but you can also set it here if your GPU/driver is problematic.
    #
    # set_sw_render = SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value=TextSubstitution(text="1"))

    # Gazebo classic will respect DISPLAY/Wayland settings from the environment. If GUI doesn't show up,
    # try launching with `gui:=false` and visualize in RViz2.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "verbose": "true",
            "gui": LaunchConfiguration("gui"),
            # Ensure simulation starts unpaused (so /clock ticks even without GUI).
            "pause": "false",
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
        condition=UnlessCondition(LaunchConfiguration("use_gazebo_joint_states")),
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
            gui_arg,
            use_sim_time_arg,
            use_gazebo_joint_states_arg,
            robot_x_arg,
            robot_y_arg,
            robot_z_arg,
            robot_yaw_arg,
            set_model_path,
            set_plugin_path,
            disable_model_db,
            gazebo,
            spawn_robot,
            robot_state_publisher,
            wheel_joint_state_publisher,
            environment_markers,
        ]
    )
