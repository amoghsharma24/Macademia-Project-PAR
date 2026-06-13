import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from numpy.typing import NDArray
from rclpy.node import Node
from typing import List, Dict, Tuple
from std_msgs.msg import Empty


class treeMapper(Node):
    def __init__(self) -> None:

        super().__init__("tree_mapper_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("max_radius", 1.0),
                ("min_radius", 1.0),
                ("free_threshold", 1),
                ("occupied_threshold", 1),
                ("height_bounds", 1.0),
                ("width_bounds", 1.0),
                ("input_map_topic", "/default_topic"),
                ("output_trees_topic", "/default_topic"),
            ],
        )

        # State Data
        self.most_recent_origin = None
        self.most_recent_map = None
        self.most_recent_resolution = None
        self.most_recent_time = None
        self.parameters = None
        self.started = False

        # Other Parameters
        input_map_topic = self.get_parameter("input_map_topic").value
        output_trees_topic = self.get_parameter("output_trees_topic").value

        # Publishers

        self.tree_publisher = self.create_publisher(PoseArray, output_trees_topic, 10)

        # self.single_tree_publisher = self.create_publisher(Pose, "/tree", 1)

        # Timers

        self.timer = self.create_timer(1, self.generate_from_topic)

        # Subscriptions

        self.create_subscription(OccupancyGrid, input_map_topic, self.parse_occupancy_msg, 10)

        self.create_subscription(
            Empty, "/tree_generator_start", self.run_generation, 10
        )

    # region main functions

    def transform_to_contours(
        self,
        img: NDArray[np.uint8],
    ) -> NDArray[np.float32] or None:

        contours, hierarchy = cv2.findContours(
            img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        return contours

    def check_radius(self, r: float) -> bool:

        min_radius = self.parameters["min_radius"]
        max_radius = self.parameters["max_radius"]
        if r < min_radius or r > max_radius:
            return False
        return True

    # def check_bounds(self, x: float, y: float) -> bool:

    #     width = self.parameters["width_bounds"]
    #     height = self.parameters["height_bounds"]
    #     #   2 * width
    #     #      _______
    #     #     |       |
    #     #     |       |
    #     #     |       |
    #     #     |       | height
    #     #     --[0,0]--

    #     if abs(x) > width:
    #         return False

    #     if y > height or y < 0:
    #         return False

    #     return True

    # endregion

    # region conversion methods

    def transform_grid_to_image(
        self,
        msg: OccupancyGrid,
    ) -> NDArray[np.uint8]:
        """
        Convert a ROS2 OccupancyGrid into an OpenCV binary image.

        OccupancyGrid values:
            -1  = unknown
            0   = free
            100 = occupied
        """

        width = msg.info.width
        height = msg.info.height

        # Convert flat occupancy data into numpy array
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Create binary image

        # free      -> 0   (black)
        # occupied  -> 255 (white)
        # unknown   -> 0   (black)

        img = np.zeros((height, width), dtype=np.uint8)

        free_thresh = self.parameters["free_threshold"]
        occupied_thresh = self.parameters["occupied_threshold"]

        img[(grid <= free_thresh)] = 0
        img[grid >= occupied_thresh] = 255

        return img

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

    # endregion
    # region control methods

    def run_generation(self, msg: Empty) -> None:
        self.started = True
        self.get_logger().info("start trigger recieved.")

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
            self.get_logger().info(f"""map information not recieved.""")
            # map: {self.most_recent_map is not None}
            # origin: {self.most_recent_origin is not None}
            # resolution: {self.most_recent_resolution is not None}
            # time: {self.most_recent_time is not None}
            # parameters: {self.parameters is not None}""")
            return

        self.get_logger().info("starting information & start trigger recieved.")
        contours = self.transform_to_contours(self.most_recent_map)

        if contours is None or len(contours) == 0:
            self.get_logger().info("No contours found.")
            return

        trees = []
        for c in contours:
            (x, y), r = cv2.minEnclosingCircle(c)
            # convert to meters
            x_meters = (
                x * self.most_recent_resolution + self.most_recent_origin["origin_x"]
            )
            y_meters = (
                y * self.most_recent_resolution + self.most_recent_origin["origin_y"]
            )
            r_meters = r * self.most_recent_resolution

            # if not self.check_bounds(x_meters, y_meters):

            #     continue

            if not self.check_radius(r_meters):
                continue
            # self.publish_circle(x_meters, y_meters, r_meters)
            trees.append((x_meters, y_meters))

        self.get_logger().info("trees processed")

        if len(trees) == 0:
            self.get_logger().info("all trees were filtered out")
            return None

        self.get_logger().info(f"trees detected: {len(trees)}")

        self.get_logger().info("2/3 trees transformed")
        self.publish_circles(trees, self.most_recent_time)
        self.get_logger().info("3/3 trees published")

    # endregion
    # region subscribers
    def parse_occupancy_msg(
        self,
        msg: OccupancyGrid,
    ) -> None:

        if not self.started:
            return
        # self.get_logger().info("grid recieved, processing info")

        self.parameters = self.get_parameters_dict()
        self.most_recent_origin = self.get_origin(msg)
        self.most_recent_map = self.transform_grid_to_image(msg)
        self.most_recent_resolution = msg.info.resolution
        self.most_recent_time = msg.info.map_load_time

        # self.get_logger().info("info processed")

    # endregion

    # region value grabbers

    def get_parameters_dict(self):
        params_dict = {
            "min_radius": self.get_parameter("min_radius").value,
            "max_radius": self.get_parameter("max_radius").value,
            "free_threshold": self.get_parameter("free_threshold").value,
            "occupied_threshold": self.get_parameter("occupied_threshold").value,
            "height_bounds": self.get_parameter("height_bounds").value,
            "width_bounds": self.get_parameter("width_bounds").value,
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

        trees.header.frame_id = "map"

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
        tree = PoseStamped()
        tree.header.frame_id = "map"
        tree.header.stamp = self.most_recent_time
        tree.pose.position.x = float(x)
        tree.pose.position.y = float(y)
        tree.pose.position.z = 0.0

        tree.pose.orientation.x = 0.0
        tree.pose.orientation.y = 0.0
        tree.pose.orientation.z = 0.0
        tree.pose.orientation.w = 1.0

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
