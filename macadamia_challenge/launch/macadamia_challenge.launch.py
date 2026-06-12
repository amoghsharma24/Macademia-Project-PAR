from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='macadamia_challenge',
            executable='spiral_controller',
            name='spiral_controller',
            output='screen',
            parameters=[{
                'center_x': 0.0,
                'center_y': 0.0,
                'min_radius': 0.5,
                'max_radius': 4.0,
                'loop_spacing': 0.25,
                'linear_speed': 0.12,
                'kp_heading': 1.5,
            }]
        )
    ])