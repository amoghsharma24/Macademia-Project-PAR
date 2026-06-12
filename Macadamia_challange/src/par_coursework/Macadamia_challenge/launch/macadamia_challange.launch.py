from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # region launch arguments
            DeclareLaunchArgument("frame_id", default_value="odom"),
            DeclareLaunchArgument("use_fake_trees", default_value="false"),
            DeclareLaunchArgument("use_tree_mapper", default_value="true"),
            DeclareLaunchArgument("use_boundary_filter", default_value="false"),
            DeclareLaunchArgument("use_memory", default_value="false"),
            DeclareLaunchArgument("use_nav2_sender", default_value="true"),
            DeclareLaunchArgument("use_orchard_controller", default_value="true"),
            DeclareLaunchArgument("use_spiral_controller", default_value="true"),
            DeclareLaunchArgument("start_active", default_value="true"),
            DeclareLaunchArgument("nav2_start_active", default_value="false"),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("nav2_auto_send", default_value="true"),
            DeclareLaunchArgument("use_hardcoded_trees", default_value="false"),
            DeclareLaunchArgument("waypoint_mode", default_value="towards_centerline"),
            DeclareLaunchArgument("waypoint_offset_x", default_value="0.0"),
            DeclareLaunchArgument("waypoint_offset_y", default_value="-0.6"),
            DeclareLaunchArgument("centreline_y", default_value="0.0"),
            DeclareLaunchArgument("approach_distance", default_value="0.6"),
            DeclareLaunchArgument("min_x", default_value="0.0"),
            DeclareLaunchArgument("max_x", default_value="5.0"),
            DeclareLaunchArgument("min_y", default_value="-2.5"),
            DeclareLaunchArgument("max_y", default_value="2.5"),
            DeclareLaunchArgument("outside_value", default_value="0"),
            DeclareLaunchArgument("tree_mapper_input_map_topic", default_value="/map"),
            DeclareLaunchArgument(
                "tree_mapper_output_topic", default_value="/detected_trees"
            ),
            DeclareLaunchArgument("tree_mapper_min_radius", default_value="0.05"),
            DeclareLaunchArgument("tree_mapper_max_radius", default_value="0.5"),
            DeclareLaunchArgument(
                "radius_min",
                default_value="-1.0",
                description="Minimum circle radius to detect (meters)",
            ),
            DeclareLaunchArgument(
                "radius_max",
                default_value="5.0",
                description="Maximum circle radius to detect (meters)",
            ),
            DeclareLaunchArgument(
                "threshold_free",
                default_value="25",
                description="Integer probability maximum for a occupancy grid square to be considered free (1-100)",
            ),
            DeclareLaunchArgument(
                "threshold_occupied",
                default_value="65",
                description="Integer probability minimum for a occupancy grid square to be considered occupied (1-100)",
            ),
            DeclareLaunchArgument(
                "bounds_height",
                default_value="30.0",
                description="the maximum distance infront of the starting location where detected trees will be considered. (meters)",
            ),
            DeclareLaunchArgument(
                "bounds_width",
                default_value="15.0",
                description="the maximum distance at either side of the starting location where detected trees will be considered.",
            ),
            # endregion
            # region nodes
            DeclareLaunchArgument("spiral_min_radius", default_value="0.25"),
            DeclareLaunchArgument("spiral_max_radius", default_value="1.4"),
            DeclareLaunchArgument("spiral_loop_spacing", default_value="1.0"),
            DeclareLaunchArgument("spiral_mode", default_value="steering"),
            DeclareLaunchArgument("spiral_linear_speed", default_value="0.125"),
            DeclareLaunchArgument("spiral_kp_heading", default_value="1.5"),
            DeclareLaunchArgument(
                "spiral_nav2_action_name", default_value="navigate_through_poses"
            ),
            DeclareLaunchArgument("spiral_nav2_goal_frame", default_value="map"),
            DeclareLaunchArgument(
                "spiral_nav2_odom_topic", default_value="/odometry/filtered"
            ),
            DeclareLaunchArgument("spiral_nav2_waypoint_spacing", default_value="0.35"),
            DeclareLaunchArgument("spiral_nav2_batch_size", default_value="8"),
            Node(
                package="macadamia_challenge",
                executable="fake_tree_publisher_node",
                name="fake_tree_publisher",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_fake_trees")),
                parameters=[
                    {"frame_id": LaunchConfiguration("frame_id")},
                ],
            ),
            Node(
                package="macadamia_challenge",
                executable="fake_boundary_filter_node",
                name="fake_boundary_filter",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_boundary_filter")),
                parameters=[
                    {"frame_id": LaunchConfiguration("frame_id")},
                    {"min_x": LaunchConfiguration("min_x")},
                    {"max_x": LaunchConfiguration("max_x")},
                    {"min_y": LaunchConfiguration("min_y")},
                    {"max_y": LaunchConfiguration("max_y")},
                    {"outside_value": LaunchConfiguration("outside_value")},
                    {"start_active": LaunchConfiguration("start_active")},
                ],
            ),
            Node(
                package="macadamia_challenge",
                executable="tree_mapper_node",
                name="tree_mapper_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_tree_mapper")),
                parameters=[
                    # {
                    #     "input_map_topic": LaunchConfiguration(
                    #         "tree_mapper_input_map_topic"
                    #     )
                    # },
                    # {
                    #     "output_trees_topic": LaunchConfiguration(
                    #         "tree_mapper_output_topic"
                    #     )
                    # },
                    # {"frame_id": LaunchConfiguration("frame_id")},
                    {
                        "max_radius": LaunchConfiguration("radius_max"),
                        "min_radius": LaunchConfiguration("radius_min"),
                        "free_threshold": LaunchConfiguration("threshold_free"),
                        "occupied_threshold": LaunchConfiguration("threshold_occupied"),
                        "height_bounds": LaunchConfiguration("bounds_height"),
                        "width_bounds": LaunchConfiguration("bounds_width"),
                    },
                ],
            ),
            Node(
                package="macadamia_challenge",
                executable="tree_memory_node",
                name="tree_memory",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_memory")),
                parameters=[
                    {"frame_id": LaunchConfiguration("frame_id")},
                    {"start_active": LaunchConfiguration("start_active")},
                ],
            ),
            Node(
                package="macadamia_challenge",
                executable="tree_waypoint_planner_node",
                name="tree_waypoint_planner",
                output="screen",
                parameters=[
                    {"frame_id": LaunchConfiguration("frame_id")},
                    {"use_hardcoded_trees": LaunchConfiguration("use_hardcoded_trees")},
                    {"waypoint_mode": LaunchConfiguration("waypoint_mode")},
                    {"waypoint_offset_x": LaunchConfiguration("waypoint_offset_x")},
                    {"waypoint_offset_y": LaunchConfiguration("waypoint_offset_y")},
                    {"centreline_y": LaunchConfiguration("centreline_y")},
                    {"approach_distance": LaunchConfiguration("approach_distance")},
                    {"start_active": LaunchConfiguration("start_active")},
                ],
            ),
            Node(
                package="macadamia_challenge",
                executable="nav2_waypoint_sender_node",
                name="nav2_waypoint_sender",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_nav2_sender")),
                parameters=[
                    {"auto_send": LaunchConfiguration("nav2_auto_send")},
                    {"start_active": LaunchConfiguration("nav2_start_active")},
                ],
            ),
            Node(
                package="macadamia_challenge",
                executable="orchard_control_node",
                name="orchard_control_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_orchard_controller")),
                parameters=[
                    {"auto_start": LaunchConfiguration("auto_start")},
                    {"frame_id": LaunchConfiguration("frame_id")},
                    {"spiral_min_radius": LaunchConfiguration("spiral_min_radius")},
                    {"spiral_max_radius": LaunchConfiguration("spiral_max_radius")},
                    {"spiral_loop_spacing": LaunchConfiguration("spiral_loop_spacing")},
                ],
            ),
            Node(
                package="macadamia_challenge",
                executable="spiral_controller",
                name="spiral_steering_controller",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("use_spiral_controller"),
                            "' == 'true' and ",
                            "'",
                            LaunchConfiguration("spiral_mode"),
                            "' == 'steering'",
                        ]
                    )
                ),
                parameters=[
                    {"min_radius": LaunchConfiguration("spiral_min_radius")},
                    {"max_radius": LaunchConfiguration("spiral_max_radius")},
                    {"loop_spacing": LaunchConfiguration("spiral_loop_spacing")},
                    {"linear_speed": LaunchConfiguration("spiral_linear_speed")},
                    {"kp_heading": LaunchConfiguration("spiral_kp_heading")},
                ],
            ),
            Node(
                package="macadamia_challenge",
                executable="spiral_nav2_controller",
                name="spiral_nav2_controller",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("use_spiral_controller"),
                            "' == 'true' and ",
                            "'",
                            LaunchConfiguration("spiral_mode"),
                            "' == 'nav2'",
                        ]
                    )
                ),
                parameters=[
                    {"min_radius": LaunchConfiguration("spiral_min_radius")},
                    {"max_radius": LaunchConfiguration("spiral_max_radius")},
                    {"loop_spacing": LaunchConfiguration("spiral_loop_spacing")},
                    {
                        "nav2_action_name": LaunchConfiguration(
                            "spiral_nav2_action_name"
                        )
                    },
                    {"goal_frame": LaunchConfiguration("spiral_nav2_goal_frame")},
                    {"odom_topic": LaunchConfiguration("spiral_nav2_odom_topic")},
                    {
                        "waypoint_spacing": LaunchConfiguration(
                            "spiral_nav2_waypoint_spacing"
                        )
                    },
                    {"batch_size": LaunchConfiguration("spiral_nav2_batch_size")},
                ],
            ),
            # end region
        ]
    )
