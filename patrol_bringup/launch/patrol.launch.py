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
    obstacle_slow_distance_arg = DeclareLaunchArgument("obstacle_slow_distance", default_value="0.90")
    obstacle_clear_distance_arg = DeclareLaunchArgument("obstacle_clear_distance", default_value="0.65")
    avoid_min_turn_time_arg = DeclareLaunchArgument("avoid_min_turn_time", default_value="0.35")
    avoid_clear_hold_time_arg = DeclareLaunchArgument("avoid_clear_hold_time", default_value="0.15")
    avoid_dir_hysteresis_arg = DeclareLaunchArgument("avoid_dir_hysteresis", default_value="0.08")
    scan_front_angle_arg = DeclareLaunchArgument("scan_front_angle", default_value="0.60")
    scan_side_angle_arg = DeclareLaunchArgument("scan_side_angle", default_value="1.20")
    goal_min_weight_arg = DeclareLaunchArgument("goal_min_weight", default_value="0.25")
    goal_blend_distance_arg = DeclareLaunchArgument("goal_blend_distance", default_value="2.0")
    scan_steer_gain_arg = DeclareLaunchArgument("scan_steer_gain", default_value="1.6")
    dwell_time_arg = DeclareLaunchArgument("dwell_time", default_value="2.0")
    loop_patrol_arg = DeclareLaunchArgument("loop_patrol", default_value="true")
    vision_roi_size_arg = DeclareLaunchArgument("vision_roi_size", default_value="80")
    vision_dominance_ratio_arg = DeclareLaunchArgument("vision_dominance_ratio", default_value="1.25")
    vision_min_channel_value_arg = DeclareLaunchArgument("vision_min_channel_value", default_value="60")
    vision_min_pixel_fraction_arg = DeclareLaunchArgument("vision_min_pixel_fraction", default_value="0.06")
    vision_publish_interval_arg = DeclareLaunchArgument("vision_publish_interval", default_value="0.2")
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
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "roi_size": ParameterValue(
                            LaunchConfiguration("vision_roi_size"), value_type=int
                        ),
                        "dominance_ratio": ParameterValue(
                            LaunchConfiguration("vision_dominance_ratio"), value_type=float
                        ),
                        "min_channel_value": ParameterValue(
                            LaunchConfiguration("vision_min_channel_value"), value_type=int
                        ),
                        "min_pixel_fraction": ParameterValue(
                            LaunchConfiguration("vision_min_pixel_fraction"), value_type=float
                        ),
                        "publish_interval": ParameterValue(
                            LaunchConfiguration("vision_publish_interval"), value_type=float
                        ),
                    }
                ],
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
                        "obstacle_slow_distance": ParameterValue(
                            LaunchConfiguration("obstacle_slow_distance"), value_type=float
                        ),
                        "obstacle_clear_distance": ParameterValue(
                            LaunchConfiguration("obstacle_clear_distance"), value_type=float
                        ),
                        "avoid_min_turn_time": ParameterValue(
                            LaunchConfiguration("avoid_min_turn_time"), value_type=float
                        ),
                        "avoid_clear_hold_time": ParameterValue(
                            LaunchConfiguration("avoid_clear_hold_time"), value_type=float
                        ),
                        "avoid_dir_hysteresis": ParameterValue(
                            LaunchConfiguration("avoid_dir_hysteresis"), value_type=float
                        ),
                        "scan_front_angle": ParameterValue(
                            LaunchConfiguration("scan_front_angle"), value_type=float
                        ),
                        "scan_side_angle": ParameterValue(
                            LaunchConfiguration("scan_side_angle"), value_type=float
                        ),
                        "goal_min_weight": ParameterValue(
                            LaunchConfiguration("goal_min_weight"), value_type=float
                        ),
                        "goal_blend_distance": ParameterValue(
                            LaunchConfiguration("goal_blend_distance"), value_type=float
                        ),
                        "scan_steer_gain": ParameterValue(
                            LaunchConfiguration("scan_steer_gain"), value_type=float
                        ),
                        "dwell_time": ParameterValue(
                            LaunchConfiguration("dwell_time"), value_type=float
                        ),
                        "loop_patrol": ParameterValue(
                            LaunchConfiguration("loop_patrol"), value_type=bool
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
            obstacle_slow_distance_arg,
            obstacle_clear_distance_arg,
            avoid_min_turn_time_arg,
            avoid_clear_hold_time_arg,
            avoid_dir_hysteresis_arg,
            scan_front_angle_arg,
            scan_side_angle_arg,
            goal_min_weight_arg,
            goal_blend_distance_arg,
            scan_steer_gain_arg,
            dwell_time_arg,
            loop_patrol_arg,
            vision_roi_size_arg,
            vision_dominance_ratio_arg,
            vision_min_channel_value_arg,
            vision_min_pixel_fraction_arg,
            vision_publish_interval_arg,
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
