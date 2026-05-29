from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='odom'),
        DeclareLaunchArgument('use_hardcoded_trees', default_value='true'),
        DeclareLaunchArgument('waypoint_mode', default_value='towards_centerline'),
        DeclareLaunchArgument('waypoint_offset_x', default_value='0.0'),
        DeclareLaunchArgument('waypoint_offset_y', default_value='-0.6'),
        DeclareLaunchArgument('centreline_y', default_value='0.0'),
        DeclareLaunchArgument('approach_distance', default_value='0.6'),

        Node(
            package='tree_waypoint_planner',
            executable='tree_waypoint_planner_node',
            name='tree_waypoint_planner',
            output='screen',
            parameters=[
                {'frame_id': LaunchConfiguration('frame_id')},
                {'use_hardcoded_trees': LaunchConfiguration('use_hardcoded_trees')},
                {'waypoint_mode': LaunchConfiguration('waypoint_mode')},
                {'waypoint_offset_x': LaunchConfiguration('waypoint_offset_x')},
                {'waypoint_offset_y': LaunchConfiguration('waypoint_offset_y')},
                {'centreline_y': LaunchConfiguration('centreline_y')},
                {'approach_distance': LaunchConfiguration('approach_distance')},
            ],
        ),
    ])
