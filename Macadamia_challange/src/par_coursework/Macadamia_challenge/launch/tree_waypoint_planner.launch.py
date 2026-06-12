from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='odom'),

        DeclareLaunchArgument('use_fake_trees', default_value='false'),
        DeclareLaunchArgument('use_boundary_filter', default_value='false'),
        DeclareLaunchArgument('use_memory', default_value='false'),
        DeclareLaunchArgument('use_nav2_sender', default_value='true'),
        DeclareLaunchArgument('use_orchard_controller', default_value='true'),
        DeclareLaunchArgument('use_spiral_controller', default_value='true'),

        DeclareLaunchArgument('start_active', default_value='true'),
        DeclareLaunchArgument('nav2_start_active', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('nav2_auto_send', default_value='true'),

        DeclareLaunchArgument('use_hardcoded_trees', default_value='true'),
        DeclareLaunchArgument('waypoint_mode', default_value='towards_centerline'),
        DeclareLaunchArgument('waypoint_offset_x', default_value='0.0'),
        DeclareLaunchArgument('waypoint_offset_y', default_value='-0.6'),
        DeclareLaunchArgument('centreline_y', default_value='0.0'),
        DeclareLaunchArgument('approach_distance', default_value='0.6'),

        DeclareLaunchArgument('min_x', default_value='0.0'),
        DeclareLaunchArgument('max_x', default_value='5.0'),
        DeclareLaunchArgument('min_y', default_value='-2.5'),
        DeclareLaunchArgument('max_y', default_value='2.5'),
        DeclareLaunchArgument('outside_value', default_value='0'),

        DeclareLaunchArgument('spiral_min_radius', default_value='0.25'),
        DeclareLaunchArgument('spiral_max_radius', default_value='1.4'),
        DeclareLaunchArgument('spiral_loop_spacing', default_value='1.0'),
        DeclareLaunchArgument('spiral_linear_speed', default_value='0.125'),
        DeclareLaunchArgument('spiral_kp_heading', default_value='1.5'),

        Node(
            package='macadamia_challenge',
            executable='fake_tree_publisher_node',
            name='fake_tree_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_fake_trees')),
            parameters=[
                {'frame_id': LaunchConfiguration('frame_id')},
            ],
        ),
        Node(
            package='macadamia_challenge',
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
            package='macadamia_challenge',
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
            package='macadamia_challenge',
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
                {'start_active': LaunchConfiguration('start_active')},
            ],
        ),
        Node(
            package='macadamia_challenge',
            executable='nav2_waypoint_sender_node',
            name='nav2_waypoint_sender',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_nav2_sender')),
            parameters=[
                {'auto_send': LaunchConfiguration('nav2_auto_send')},
                {'start_active': LaunchConfiguration('nav2_start_active')},
            ],
        ),
        Node(
            package='macadamia_challenge',
            executable='orchard_control_node',
            name='orchard_control_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_orchard_controller')),
            parameters=[
                {'auto_start': LaunchConfiguration('auto_start')},
                {'frame_id': LaunchConfiguration('frame_id')},
                {'spiral_min_radius': LaunchConfiguration('spiral_min_radius')},
                {'spiral_max_radius': LaunchConfiguration('spiral_max_radius')},
                {'spiral_loop_spacing': LaunchConfiguration('spiral_loop_spacing')},
            ],
        ),
        Node(
            package='macadamia_challenge',
            executable='spiral_controller',
            name='spiral_controller',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_spiral_controller')),
            parameters=[
                {'min_radius': LaunchConfiguration('spiral_min_radius')},
                {'max_radius': LaunchConfiguration('spiral_max_radius')},
                {'loop_spacing': LaunchConfiguration('spiral_loop_spacing')},
                {'linear_speed': LaunchConfiguration('spiral_linear_speed')},
                {'kp_heading': LaunchConfiguration('spiral_kp_heading')},
            ],
        ),
    ])
