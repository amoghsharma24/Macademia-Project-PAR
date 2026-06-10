from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'tree_waypoint_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'fake_tree_publisher_node = tree_waypoint_planner.fake_tree_publisher_node:main',
            'fake_boundary_filter_node = tree_waypoint_planner.fake_boundary_filter_node:main',
            'tree_memory_node = tree_waypoint_planner.tree_memory_node:main',
            'tree_waypoint_planner_node = tree_waypoint_planner.tree_waypoint_planner_node:main',
            'nav2_waypoint_sender_node = tree_waypoint_planner.nav2_waypoint_sender_node:main',
        ],
    },
)
