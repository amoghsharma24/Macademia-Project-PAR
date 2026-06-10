from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('use_boundary_filter', default_value='true'),
        DeclareLaunchArgument('use_memory', default_value='true'),
        DeclareLaunchArgument('use_hardcoded_trees', default_value='false'),
        DeclareLaunchArgument('auto_send', default_value='false'),
        DeclareLaunchArgument('start_active', default_value='true'),
        DeclareLaunchArgument('min_x', default_value='0.0'),
        DeclareLaunchArgument('max_x', default_value='5.0'),
        DeclareLaunchArgument('min_y', default_value='-2.5'),
        DeclareLaunchArgument('max_y', default_value='2.5'),
        DeclareLaunchArgument('outside_value', default_value='0'),
        DeclareLaunchArgument('waypoint_mode', default_value='towards_centerline'),
        DeclareLaunchArgument('centreline_y', default_value='0.0'),
        DeclareLaunchArgument('approach_distance', default_value='0.9'),

        Node(
            package='tree_waypoint_planner',
            executable='fake_boundary_filter_node',
            name='fake_boundary_filter',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_boundary_filter')),
            parameters=[
                {'frame_id': LaunchConfiguration('frame_id')},
                {'min_x': LaunchConfiguration('min_x')},
                {'max_x': LaunchConfiguration('max_x')},
                {'min_y': LaunchConfiguration('min_y')},
                {'max_y': LaunchConfiguration('max_y')},
                {'outside_value': LaunchConfiguration('outside_value')},
                {'start_active': LaunchConfiguration('start_active')},
            ],
        ),
        Node(
            package='tree_waypoint_planner',
            executable='tree_memory_node',
            name='tree_memory',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_memory')),
            parameters=[
                {'frame_id': LaunchConfiguration('frame_id')},
                {'start_active': LaunchConfiguration('start_active')},
            ],
        ),
        Node(
            package='tree_waypoint_planner',
            executable='tree_waypoint_planner_node',
            name='tree_waypoint_planner',
            output='screen',
            parameters=[
                {'frame_id': LaunchConfiguration('frame_id')},
                {'use_hardcoded_trees': LaunchConfiguration('use_hardcoded_trees')},
                {'waypoint_mode': LaunchConfiguration('waypoint_mode')},
                {'centreline_y': LaunchConfiguration('centreline_y')},
                {'approach_distance': LaunchConfiguration('approach_distance')},
                {'start_active': LaunchConfiguration('start_active')},
            ],
        ),
        Node(
            package='tree_waypoint_planner',
            executable='nav2_waypoint_sender_node',
            name='nav2_waypoint_sender',
            output='screen',
            parameters=[
                {'auto_send': LaunchConfiguration('auto_send')},
                {'start_active': LaunchConfiguration('start_active')},
            ],
        ),
    ])
