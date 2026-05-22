from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('use_hardcoded_trees', default_value='true'),
        DeclareLaunchArgument('waypoint_offset_x', default_value='0.0'),
        DeclareLaunchArgument('waypoint_offset_y', default_value='-0.6'),
        DeclareLaunchArgument('tree_marker_radius', default_value='0.25'),
        DeclareLaunchArgument('tree_marker_height', default_value='0.8'),

        Node(
            package='tree_waypoint_planner',
            executable='tree_waypoint_planner_node',
            name='tree_waypoint_planner',
            output='screen',
            parameters=[
                {'frame_id': LaunchConfiguration('frame_id')},
                {'use_hardcoded_trees': LaunchConfiguration('use_hardcoded_trees')},
                {'waypoint_offset_x': LaunchConfiguration('waypoint_offset_x')},
                {'waypoint_offset_y': LaunchConfiguration('waypoint_offset_y')},
                {'tree_marker_radius': LaunchConfiguration('tree_marker_radius')},
                {'tree_marker_height': LaunchConfiguration('tree_marker_height')},
            ],
        ),
    ])
