from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("patrol_bringup")
    nav2_share = FindPackageShare("nav2_bringup")

    use_nav2_arg = DeclareLaunchArgument("use_nav2", default_value="false")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    max_linear_arg = DeclareLaunchArgument("max_linear", default_value="0.35")
    obstacle_stop_distance_arg = DeclareLaunchArgument("obstacle_stop_distance", default_value="0.45")
    use_path_planner_arg = DeclareLaunchArgument("use_path_planner", default_value="true")
    grid_resolution_arg = DeclareLaunchArgument("grid_resolution", default_value="0.10")
    wall_inflation_arg = DeclareLaunchArgument("wall_inflation", default_value="0.25")
    lookahead_distance_arg = DeclareLaunchArgument("lookahead_distance", default_value="0.7")
    plan_interval_arg = DeclareLaunchArgument("plan_interval", default_value="1.0")
    obstacle_a_x_arg = DeclareLaunchArgument("obstacle_a_x", default_value="3.974")
    obstacle_a_y_arg = DeclareLaunchArgument("obstacle_a_y", default_value="5.786")
    obstacle_b_x_arg = DeclareLaunchArgument("obstacle_b_x", default_value="4.024")
    obstacle_b_y_arg = DeclareLaunchArgument("obstacle_b_y", default_value="3.836")
    obstacle_speed_arg = DeclareLaunchArgument("obstacle_speed", default_value="0.3")
    nav2_params_arg = DeclareLaunchArgument(
        "nav2_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "nav2_params.yaml"]),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    control_nodes = GroupAction(
        actions=[
            Node(
                package="patrol_control",
                executable="vision_checker",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="patrol_control",
                executable="scan_to_range",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="patrol_control",
                executable="obstacle_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "point_a_x": ParameterValue(
                            LaunchConfiguration("obstacle_a_x"), value_type=float
                        ),
                        "point_a_y": ParameterValue(
                            LaunchConfiguration("obstacle_a_y"), value_type=float
                        ),
                        "point_b_x": ParameterValue(
                            LaunchConfiguration("obstacle_b_x"), value_type=float
                        ),
                        "point_b_y": ParameterValue(
                            LaunchConfiguration("obstacle_b_y"), value_type=float
                        ),
                        "speed": ParameterValue(
                            LaunchConfiguration("obstacle_speed"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="patrol_control",
                executable="patrol_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "max_linear": ParameterValue(
                            LaunchConfiguration("max_linear"), value_type=float
                        ),
                        "obstacle_stop_distance": ParameterValue(
                            LaunchConfiguration("obstacle_stop_distance"), value_type=float
                        ),
                        "use_path_planner": ParameterValue(
                            LaunchConfiguration("use_path_planner"), value_type=bool
                        ),
                        "grid_resolution": ParameterValue(
                            LaunchConfiguration("grid_resolution"), value_type=float
                        ),
                        "wall_inflation": ParameterValue(
                            LaunchConfiguration("wall_inflation"), value_type=float
                        ),
                        "lookahead_distance": ParameterValue(
                            LaunchConfiguration("lookahead_distance"), value_type=float
                        ),
                        "plan_interval": ParameterValue(
                            LaunchConfiguration("plan_interval"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )

    # Optional Nav2 bringup (requires additional setup: localization, map, etc.)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, "launch", "bringup_launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("nav2_params"),
        }.items(),
    )

    return LaunchDescription(
        [
            use_nav2_arg,
            use_sim_time_arg,
            max_linear_arg,
            obstacle_stop_distance_arg,
            use_path_planner_arg,
            grid_resolution_arg,
            wall_inflation_arg,
            lookahead_distance_arg,
            plan_interval_arg,
            obstacle_a_x_arg,
            obstacle_a_y_arg,
            obstacle_b_x_arg,
            obstacle_b_y_arg,
            obstacle_speed_arg,
            nav2_params_arg,
            gazebo,
            nav2,
            control_nodes,
        ]
    )
