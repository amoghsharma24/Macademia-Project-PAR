from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    test = "blah blah blah"
    return LaunchDescription(
        [
            # region
            # launch arguments
            SetEnvironmentVariable("TEST", "1"),
            DeclareLaunchArgument(
                "maps_path",
                default_value=[
                    # "/ros2_ws/src/par_coursework/tree_mapper/maps",
                    "./src/par_coursework/tree_mapper/maps",
                    # use when running in vm.
                    # "/humble_workspace/src/par_coursework/objects_example",
                ],
                description="Directory containing maps to detect trees for.",
            ),
            # DeclareLaunchArgument(
            #     "hough_dp",
            #     default_value="1.0",
            #     description="Inverse ratio of accumulator resolution to image resolution.",
            # ),
            # DeclareLaunchArgument(
            #     "hough_min_dist",
            #     default_value=".2",
            #     description="Minimum distance between detected circle centers (meters)",
            # ),
            # DeclareLaunchArgument(
            #     "hough_param1",
            #     default_value="10.0",
            #     description="Higher threshold for the internal Canny edge detector.",
            # ),
            # DeclareLaunchArgument(
            #     "hough_param2",
            #     default_value="10.0",
            #     description="Accumulator threshold for circle detection.",
            # ),
            # DeclareLaunchArgument(
            #     "hough_min_radius",
            #     default_value="0.05",
            #     description="Minimum circle radius to detect (meters)",
            # ),
            # DeclareLaunchArgument(
            #     "hough_max_radius",
            #     default_value="0.09",
            #     description="Maximum circle radius to detect (meters)",
            # ),
            DeclareLaunchArgument(
                "radius_min",
                default_value="-1.0",
                description="Minimum circle radius to detect (pixels)",
            ),
            DeclareLaunchArgument(
                "radius_max",
                default_value="5.0",
                description="Maximum circle radius to detect (pixels)",
            ),
            # endregion
            # region nodes
            Node(
                package="tree_mapper",
                executable="tree_mapper_node",
                name="tree_mapper_node",
                output="screen",
                parameters=[
                    {
                        "image_path": LaunchConfiguration("maps_path"),
                        # "hough.dp": LaunchConfiguration("hough_dp"),
                        # "hough.min_dist": LaunchConfiguration("hough_min_dist"),
                        # "hough.param1": LaunchConfiguration("hough_param1"),
                        # "hough.param2": LaunchConfiguration("hough_param2"),
                        # "hough.min_radius": LaunchConfiguration("hough_min_radius"),
                        # "hough.max_radius": LaunchConfiguration("hough_max_radius"),
                        "max_radius": LaunchConfiguration("radius_max"),
                        "min_radius": LaunchConfiguration("radius_min"),
                    }
                ],
            ),
            # endregion
        ]
    )
