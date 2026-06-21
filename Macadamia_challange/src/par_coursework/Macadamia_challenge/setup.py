from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'macadamia_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('macadamia_challenge/maps/*')),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RMIT AI Innovation Lab',
    description='Tree approach waypoint planner for orchard harvesting demos.',
    license='RMIT IP - Not for public release',
    entry_points={
        'console_scripts': [
            'fake_tree_publisher_node = macadamia_challenge.waypoint_provider.fake_tree_publisher_node:main',
            'fake_boundary_filter_node = macadamia_challenge.waypoint_provider.fake_boundary_filter_node:main',
            'tree_memory_node = macadamia_challenge.waypoint_provider.tree_memory_node:main',
            'tree_waypoint_planner_node = macadamia_challenge.waypoint_provider.tree_waypoint_planner_node:main',
            # 'nav2_waypoint_sender_node = macadamia_challenge.waypoint_provider.nav2_waypoint_sender_node:main',
            'path_sender_node = macadamia_challenge.path_navigator.nav2_path_sender_node:main',
            'orchard_control_node = macadamia_challenge.controll.orchard_state_controller:main',
            'orchard_control_node_direct_neighbours = macadamia_challenge.controll.orchard_state_controller_direct_neighbours:main',
            'orchard_control_node_two_closest = macadamia_challenge.controll.orchard_state_controller_two_closest:main',
            'spiral_controller = macadamia_challenge.tree_behaviour.spiral_controller:main',
            'spiral_controller_square = macadamia_challenge.tree_behaviour.spiral_controller_square:main',
            'spiral_nav2_controller = macadamia_challenge.tree_behaviour.spiral_controller_nav2_waypoint:main',
            'spiral_nav2_controller_square = macadamia_challenge.tree_behaviour.spiral_controller_nav2_waypoint_square:main',
            'tree_mapper_node = macadamia_challenge.tree_detection.tree_mapper_node:main',
        ],
    },
)
