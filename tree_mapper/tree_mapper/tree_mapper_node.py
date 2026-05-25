# 1. load map
# 2. RANSAC to detect circles.
# 3. Filter circles (?)
# 4. Publish circles

# one shot, no history.

# current goal: load occupancy grid.

# from rclpy.action import ActionClient
# from geometry_msgs.msg import Posetamped
# from nav_msgs.msg import Path
# from std_msgs.msg import String, Empty
# from nav2_msgs.action import FollowWaypoints
# import tf2_ros
# from tf2_ros import TransformException
# import math
import cv2
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from numpy.typing import NDArray
from rclpy.node import Node
from typing import List, Dict, Tuple


class treeMapper(Node):
    def __init__(self) -> None:
        super().__init__("tree_mapper_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("image_path", "."),
                ("hough.dp", 1.0),
                ("hough.min_dist", 1.0),
                ("hough.param1", 1.0),
                ("hough.param2", 1.0),
                ("hough.min_radius", 1.0),
                ("hough.max_radius", 1.0),
            ],
        )

        # State Data
        self.most_recent_origin = None
        self.most_recent_map = None
        self.most_recent_resolution = None
        self.most_recent_time = None

        # Publishers
        self.tree_publisher = self.create_publisher(PoseArray, "/trees", 10)

        # TF
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timers
        self.timer = self.create_timer(2, self.generate_from_topic)

        # Parameters
        self.map_path = self.get_parameter("image_path").value

        # Nav2 client
        # self.waypoint_client = ActionClient(self, FollowWaypoints, "/follow_waypoints")

        # Subscriptions
        self.create_subscription(OccupancyGrid, "/map", self.parse_occupancy_msg, 10)

        # Wait for Nav2
        # self.get_logger().info("Waiting for Nav2 waypoint server...")
        # self.waypoint_client.wait_for_server()
        # self.get_logger().info("Path Tracker Node Started")

    # region loading functions
    def load_map(self, map_path: str) -> NDArray[np.uint8]:
        img = cv2.imread(map_path + "/turtlebot_area.pgm", cv2.IMREAD_GRAYSCALE)
        # make it RGB so placed trees are seen easier.
        return img

    def load_map_yaml(self, yaml_path: str) -> Dict[str, any]:
        """
        Load a ROS2/Nav2 map YAML file.

        Returns:
            dict containing:
                image
                mode
                resolution
                origin
                negate
                occupied_thresh
                free_thresh
        """

        with open(yaml_path, "r") as file:
            data = yaml.safe_load(file)

        map_data = {
            "origin": data["origin"],
            "negate": int(data["negate"]),
            "occupied_thresh": float(data["occupied_thresh"]),
            "free_thresh": float(data["free_thresh"]),
        }

        return map_data

    # endregion
    # region main functions
    def filter_map(
        self,
        parsed_map: NDArray[np.uint8],
    ) -> NDArray[np.uint8]:
        return parsed_map

    def detect_circles(
        self,
        img: NDArray[np.uint8],
    ) -> NDArray[np.float32] | None:
        param_dict = self.get_hough_parameters()

        circles = cv2.HoughCircles(
            img,
            cv2.HOUGH_GRADIENT,
            dp=param_dict["dp"],
            minDist=param_dict["min_dist"],
            param1=param_dict["param1"],
            param2=param_dict["param2"],
            minRadius=param_dict["min_radius"],
            maxRadius=param_dict["max_radius"],
        )
        return circles

    def draw_circles(
        self,
        base_image: NDArray[np.uint8],
        circles: NDArray[np.float32] | None,
    ) -> NDArray[np.uint8] | None:
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles:
                for x, y, r in circle:
                    cv2.circle(base_image, (x, y), r, (0, 255, 0), 1)  # Circle outline
                    cv2.circle(base_image, (x, y), 1, (0, 0, 255), 2)  # Center point

            return base_image
        return None

    def save_image(
        self,
        img_path: str,
        img: NDArray[np.uint8],
    ) -> None:
        image_path = img_path + "/detected-trees.jpeg"
        cv2.imwrite(image_path, img)
        self.get_logger().info(f"image saved: {image_path}")

    # endregion

    # region conversion methods
    def convert_parameters_to_pixels(self, parameters: list):
        newParams = []
        for p in parameters:
            newParams.append(int(p * self.most_recent_resolution))
        return newParams

    def convert_grid_to_image(
        self,
        msg: OccupancyGrid,
    ) -> NDArray[np.uint8]:
        """
        Convert a ROS2 OccupancyGrid into an OpenCV grayscale image.

        OccupancyGrid values:
            -1   = unknown
            0   = free
            100 = occupied
        """

        width = msg.info.width
        height = msg.info.height

        # Convert flat occupancy data into numpy array
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Create grayscale image
        # free      -> 255 (white)
        # occupied  -> 0   (black)
        # unknown   -> 127 (gray)
        img = np.zeros((height, width), dtype=np.uint8)

        img[grid <= 0] = 255
        img[grid == 100] = 0
        img[grid == -1] = 127

        # Flip vertically so map orientation matches RViz/world coordinates
        # img = cv2.flip(img, 0)

        # Store image for later use
        return img

    def convert_circles_to_map(
        self,
        origin: Dict[str, float],
        resolution: float,
        circles: NDArray[np.float32],
    ) -> List[Tuple[float, float, float]]:
        # 1. convert to correct resolution
        # 2. convert to map frame
        # - is positive x forward? is positive y left?
        # 3. parse data into markers
        # 4. publish markers
        circles_scaled = circles * resolution

        circles_translated = []
        for circle in circles_scaled:
            for x, y, r in circle:
                x_meters = x + origin["origin_x"]
                y_meters = y + origin["origin_y"]
                r_meters = r
                circles_translated.append((x_meters, y_meters, r_meters))

        # self.get_logger().info(f"trees transformed: {circles_translated}")
        return circles_translated

    def get_origin(
        self,
        msg: OccupancyGrid,
    ) -> Dict[str, float]:
        info = msg.info

        metadata = {
            "origin_x": info.origin.position.x,
            "origin_y": info.origin.position.y,
            "origin_z": info.origin.position.z,
            "origin_qx": info.origin.orientation.x,
            "origin_qy": info.origin.orientation.y,
            "origin_qz": info.origin.orientation.z,
            "origin_qw": info.origin.orientation.w,
        }
        return metadata

    def generate_trees(
        self,
        map_image: NDArray[np.uint8],
    ) -> NDArray[np.float32] | None:
        output = cv2.cvtColor(map_image, cv2.COLOR_GRAY2RGB)

        map_img = self.filter_map(map_image)

        circles = self.detect_circles(map_img)

        if circles is None:
            self.get_logger().info("no trees detected")
            return None
        self.get_logger().info(f"trees detected: {len(circles[0])}")

        final_img = self.draw_circles(output, circles)

        self.save_image(self.map_path, final_img)
        return circles

    # endregion
    # region control methods

    def generate_from_saved(self) -> None:
        map_img = self.load_map(self.map_path)
        self.generate_trees(map_img)

    def generate_from_topic(self) -> None:
        if (
            self.most_recent_map is None
            or self.most_recent_origin is None
            or self.most_recent_resolution is None
            or self.most_recent_time is None
        ):
            return

        map_img = self.filter_map(self.most_recent_map)
        self.get_logger().info("1/4 map image gotten")

        circles = self.generate_trees(map_img)
        self.get_logger().info("2/4 circles gotten")

        if circles is None:
            return

        transformed_circles = self.convert_circles_to_map(
            self.most_recent_origin, self.most_recent_resolution, circles
        )
        self.get_logger().info("3/4 circles transformed")

        self.publish_circles(transformed_circles, self.most_recent_time)
        self.get_logger().info("4/4 circles published")

    # endregion
    # region subscribers
    def parse_occupancy_msg(
        self,
        msg: OccupancyGrid,
    ) -> None:
        self.most_recent_origin = self.get_origin(msg)
        self.most_recent_map = self.convert_grid_to_image(msg)
        self.most_recent_resolution = msg.info.resolution
        self.most_recent_time = msg.info.map_load_time

    # endregion

    # region value grabbers
    def get_hough_parameters(self):
        distance_params = [
            self.get_parameter("hough.min_dist").value,
            self.get_parameter("hough.min_radius").value,
            self.get_parameter("hough.max_radius").value,
        ]

        scaled_params = self.convert_parameters_to_pixels(distance_params)

        params_dict = {
            "dp": self.get_parameter("hough.dp").value,
            "min_dist": scaled_params[0],
            "param1": self.get_parameter("hough.param1").value,
            "param2": self.get_parameter("hough.param2").value,
            "min_radius": scaled_params[1],
            "max_radius": scaled_params[2],
        }
        return params_dict

    # endregion
    # region publishers
    def publish_circles(
        self,
        circles: List[Tuple[float, float, float]],
        time: Time,
    ) -> None:
        trees = PoseArray()
        trees.header.frame_id = "map"
        trees.header.stamp = time

        # Create the poses
        detected_trees = []
        for x, y, r in circles:
            pose = Pose()

            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            detected_trees.append(pose)

        trees.poses = detected_trees
        # Publish
        self.tree_publisher.publish(trees)

    # endregion


def main(args=None):
    rclpy.init(args=args)
    node = treeMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
