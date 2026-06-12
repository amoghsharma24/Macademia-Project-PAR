# 1. load map
# 2. RANSAC to detect circles.
# 3. Filter circles (?)
# 4. Publish circles

# one shot, no history.

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from numpy.typing import NDArray
from rclpy.node import Node
from typing import List, Dict, Tuple
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Empty


map_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class treeMapper(Node):
    def __init__(self) -> None:

        super().__init__("tree_mapper_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("image_path", "."),
                ("input_map_topic", "/map"),
                ("output_trees_topic", "/trees"),
                ("frame_id", "map"),
                ("max_radius", 1.0),
                ("min_radius", 1.0),
                ("free_threshold", 1),
                ("occupied_threshold", 1),
            ],
        )

        # State Data
        self.most_recent_origin = None
        self.most_recent_map = None
        self.most_recent_resolution = None
        self.most_recent_time = None
        self.parameters = None
        self.started = False

        # Publishers
        self.tree_publisher = self.create_publisher(
            PoseArray,
            self.get_parameter("output_trees_topic").value,
            10,
        )
        # self.single_tree_publisher = self.create_publisher(Pose, "/tree", 1)

        # TF
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timers
        # self.timer = self.create_timer(10, self.generate_from_topic)

        # Parameters
        self.map_path = self.get_parameter("image_path").value

        # Subscriptions
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("input_map_topic").value,
            self.parse_occupancy_msg,
            map_qos,
        )

        self.create_subscription(
            Empty, "/tree_generator_start", self.run_generation, 10
        )

    # region main functions

    def detect_contours(
        self,
        img: NDArray[np.uint8],
    ) -> NDArray[np.float32] or None:

        contours, hierarchy = cv2.findContours(
            img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        return contours

    def filter_contours(
        self, contours: NDArray[np.float32]
    ) -> List[Tuple[float, float]]:

        filtered = []

        min_radius = self.parameters["min_radius"]

        max_radius = self.parameters["max_radius"]

        for c in contours:
            # area = cv2.contourArea(c)

            (x, y), radius = cv2.minEnclosingCircle(c)

            if radius >= min_radius and radius <= max_radius:
                # self.publish_circle(x, y, radius)
                filtered.append((x, y))

        return filtered

    # endregion

    # region conversion methods
    def convert_parameters_to_pixels(self, parameters: list):
        newParams = []
        for p in parameters:
            newParams.append(int(p / self.most_recent_resolution))

        return newParams

    def convert_grid_to_image(
        self,
        msg: OccupancyGrid,
    ) -> NDArray[np.uint8]:
        """

        Convert a ROS2 OccupancyGrid into an OpenCV binary image.



        OccupancyGrid values:
            -1   = unknown
            0   = free
            100 = occupied
        """

        width = msg.info.width
        height = msg.info.height

        # Convert flat occupancy data into numpy array
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Create binary image

        # free      -> 255 (white)
        # occupied  -> 0   (black)

        # unknown   -> 0 (gray)

        img = np.zeros((height, width), dtype=np.uint8)

        img[(grid <= 25) & (grid >= 0)] = 0

        img[grid >= 65] = 255

        img[grid == -1] = 0

        return img

    def convert_circles_to_map(
        self,
        origin: Dict[str, float],
        resolution: float,
        circles: NDArray[np.float32],
    ) -> List[Tuple[float, float]]:

        # 1. convert to correct resolution
        # 2. convert to map frame
        # - is positive x forward? is positive y left?

        circles_translated = []

        for x, y in circles:
            x_meters = x * resolution + origin["origin_x"]

            y_meters = y * resolution + origin["origin_y"]

            circles_translated.append((x_meters, y_meters))

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
        self, map_image: NDArray[np.uint8]
    ) -> List[Tuple[float, float]] or None:

        contours = self.detect_contours(map_image)

        if len(contours) == 0:
            self.get_logger().info("no trees detected")
            return None

        trees = self.filter_contours(contours)

        if len(trees) == 0:
            self.get_logger().info("all trees were filtered out")
            return None

        self.get_logger().info(f"trees detected: {len(trees)}")

        return trees

    # endregion
    # region control methods
    def run_generation(self, _msg=None) -> None:
        self.started = True
        self.get_logger().info("tree generation request recieved.")
        self.generate_from_topic()

    def generate_from_topic(self) -> None:
        if not self.started:
            return

        if (
            self.most_recent_map is None
            or self.most_recent_origin is None
            or self.most_recent_resolution is None
            or self.most_recent_time is None
            or self.parameters is None
        ):
            return

        circles = self.generate_trees(self.most_recent_map)
        self.get_logger().info("1/3 trees gotten")

        if circles is None:
            return

        transformed_circles = self.convert_circles_to_map(
            self.most_recent_origin, self.most_recent_resolution, circles
        )

        self.get_logger().info("2/3 circles transformed")
        self.publish_circles(transformed_circles, self.most_recent_time)
        self.get_logger().info("3/3 circles published")

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
        self.parameters = self.get_parameters()

    # endregion

    # region value grabbers

    def get_parameters(self):

        distance_params = [
            self.get_parameter("min_radius").value,
            self.get_parameter("max_radius").value,
        ]

        scaled_params = self.convert_parameters_to_pixels(distance_params)

        params_dict = {
            "min_radius": scaled_params[0],
            "max_radius": scaled_params[1],
            "free_threshold": self.get_parameter("free_threshold").value,
            "occupied_threshold": self.get_parameter("free_threshold").value,
        }
        return params_dict

    # endregion
    # region publishers
    def publish_circles(
        self,
        circles: List[Tuple[float, float]],
        time: Time,
    ) -> None:
        trees = PoseArray()
        trees.header.frame_id = self.get_parameter("frame_id").value
        trees.header.stamp = time

        # Create the poses
        detected_trees = []

        for x, y in circles:
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

    def publish_circle(self, x, y, r) -> None:
        tree = Pose()
        tree.header.frame_id = "map"
        tree.header.stamp = self.most_recent_time
        tree.position.x = float(x)
        tree.position.y = float(y)
        tree.position.z = 0.0

        tree.orientation.x = 0.0
        tree.orientation.y = 0.0
        tree.orientation.z = 0.0
        tree.orientation.w = 1.0

        # Publish
        self.single_tree_publisher.publish(tree)

    # endregion


def main(args=None):
    rclpy.init(args=args)
    node = treeMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
