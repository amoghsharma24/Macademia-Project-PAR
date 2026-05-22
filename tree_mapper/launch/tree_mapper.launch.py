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
            SetEnvironmentVariable("TEST_1", "1"),
            DeclareLaunchArgument(
                "TEST_LAUNCH", default_value=test, description="blah blah."
            ),
            DeclareLaunchArgument(
                "maps_path",
                default_value=[
                    "/ros2_ws/src/par_coursework/tree_mapper/tree_mapper/maps",
                    # use when running in vm.
                    # "/humble_workspace/src/par_coursework/objects_example",
                ],
                description="Directory containing maps to detect trees for.",
            ),
            DeclareLaunchArgument(
                "hough_dp",
                default_value="1.0",
                description="Inverse ratio of accumulator resolution to image resolution.",
            ),
            DeclareLaunchArgument(
                "hough_min_dist",
                default_value="20.0",
                description="Minimum distance between detected circle centers.",
            ),
            DeclareLaunchArgument(
                "hough_param1",
                default_value="10.0",
                description="Higher threshold for the internal Canny edge detector.",
            ),
            DeclareLaunchArgument(
                "hough_param2",
                default_value="10.0",
                description="Accumulator threshold for circle detection.",
            ),
            DeclareLaunchArgument(
                "hough_min_radius",
                default_value="6",
                description="Minimum circle radius to detect.",
            ),
            DeclareLaunchArgument(
                "hough_max_radius",
                default_value="9",
                description="Maximum circle radius to detect.",
            ),
            # endregion
            # region
            # nodes
            Node(
                package="tree_mapper",
                executable="tree_mapper_node",
                name="tree_mapper_node",
                output="screen",
                parameters=[
                    {
                        "image_path": LaunchConfiguration("maps_path"),
                        "hough.dp": LaunchConfiguration("hough_dp"),
                        "hough.min_dist": LaunchConfiguration("hough_min_dist"),
                        "hough.param1": LaunchConfiguration("hough_param1"),
                        "hough.param2": LaunchConfiguration("hough_param2"),
                        "hough.min_radius": LaunchConfiguration("hough_min_radius"),
                        "hough.max_radius": LaunchConfiguration("hough_max_radius"),
                    }
                ],
            ),
        ]
    )
