from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    planner_type = LaunchConfiguration('planner_type')

    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_type',
            default_value='RRTstar',
            description='OMPL planner type to use'
        ),

        Node(
            package='orchard_global_planner',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[
                {
                    'planner_type': planner_type,
                }
            ],
        )
    ])