#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from visualization_msgs.msg import Marker


class FakeBoundaryFilter(Node):
    def __init__(self):
        super().__init__('fake_boundary_filter_node')

        self.declare_parameter('input_map_topic', '/map')
        self.declare_parameter('output_map_topic', '/filtered_map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('min_x', 0.0)
        self.declare_parameter('max_x', 5.0)
        self.declare_parameter('min_y', -2.0)
        self.declare_parameter('max_y', 2.0)
        self.declare_parameter('outside_value', 0)
        self.declare_parameter('publish_rate_hz', 1.0)

        input_map_topic = self.get_parameter('input_map_topic').value
        output_map_topic = self.get_parameter('output_map_topic').value

        self.latest_map = None
        self.last_log_nanoseconds = 0
        self.warned_invalid_x_boundary = False
        self.warned_invalid_y_boundary = False
        self.warned_low_outside_value = False
        self.warned_high_outside_value = False

        self.filtered_map_pub = self.create_publisher(
            OccupancyGrid,
            output_map_topic,
            10,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            '/orchard_boundary_marker',
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            input_map_topic,
            self.map_callback,
            10,
        )

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate_hz <= 0.0:
            self.get_logger().warn('publish_rate_hz must be positive; using 1.0 Hz')
            publish_rate_hz = 1.0
        self.create_timer(1.0 / publish_rate_hz, self.publish_filtered_map)

        self.get_logger().info(
            f'Fake boundary filter started: {input_map_topic} -> {output_map_topic}'
        )

    def map_callback(self, msg):
        self.latest_map = msg

    def publish_filtered_map(self):
        if self.latest_map is None:
            self.publish_boundary_marker()
            return

        filtered_map, kept_count, filtered_count = self.create_filtered_map(
            self.latest_map
        )
        self.filtered_map_pub.publish(filtered_map)
        self.publish_boundary_marker(filtered_map.header)
        self.log_filter_counts(kept_count, filtered_count)

    def create_filtered_map(self, occupancy_grid):
        filtered_map = deepcopy(occupancy_grid)
        min_x, max_x, min_y, max_y = self.get_boundary()
        outside_value = self.get_outside_value()

        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y
        filtered_data = list(occupancy_grid.data)
        kept_count = 0
        filtered_count = 0

        for cell_y in range(height):
            world_y = origin_y + (cell_y + 0.5) * resolution
            for cell_x in range(width):
                world_x = origin_x + (cell_x + 0.5) * resolution
                cell_index = cell_y * width + cell_x

                if min_x <= world_x <= max_x and min_y <= world_y <= max_y:
                    kept_count += 1
                else:
                    filtered_data[cell_index] = outside_value
                    filtered_count += 1

        filtered_map.data = filtered_data
        return filtered_map, kept_count, filtered_count

    def get_boundary(self):
        min_x = float(self.get_parameter('min_x').value)
        max_x = float(self.get_parameter('max_x').value)
        min_y = float(self.get_parameter('min_y').value)
        max_y = float(self.get_parameter('max_y').value)

        if min_x > max_x:
            if not self.warned_invalid_x_boundary:
                self.get_logger().warn(
                    'min_x is greater than max_x; swapping boundary values'
                )
                self.warned_invalid_x_boundary = True
            min_x, max_x = max_x, min_x
        if min_y > max_y:
            if not self.warned_invalid_y_boundary:
                self.get_logger().warn(
                    'min_y is greater than max_y; swapping boundary values'
                )
                self.warned_invalid_y_boundary = True
            min_y, max_y = max_y, min_y

        return min_x, max_x, min_y, max_y

    def get_outside_value(self):
        outside_value = int(self.get_parameter('outside_value').value)
        if outside_value < -128:
            if not self.warned_low_outside_value:
                self.get_logger().warn('outside_value below int8 range; using -128')
                self.warned_low_outside_value = True
            return -128
        if outside_value > 127:
            if not self.warned_high_outside_value:
                self.get_logger().warn('outside_value above int8 range; using 127')
                self.warned_high_outside_value = True
            return 127
        return outside_value

    def publish_boundary_marker(self, map_header=None):
        marker = Marker()
        if map_header is not None and map_header.frame_id:
            marker.header = map_header
        else:
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = self.get_parameter('frame_id').value

        min_x, max_x, min_y, max_y = self.get_boundary()
        marker.ns = 'fake_orchard_boundary'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [
            self.create_point(min_x, min_y),
            self.create_point(max_x, min_y),
            self.create_point(max_x, max_y),
            self.create_point(min_x, max_y),
            self.create_point(min_x, min_y),
        ]

        self.marker_pub.publish(marker)

    def create_point(self, x, y):
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.02
        return point

    def log_filter_counts(self, kept_count, filtered_count):
        now_nanoseconds = self.get_clock().now().nanoseconds
        if now_nanoseconds - self.last_log_nanoseconds < 5_000_000_000:
            return

        self.get_logger().info(
            f'Filtered map published: kept {kept_count} cells, filtered {filtered_count} cells'
        )
        self.last_log_nanoseconds = now_nanoseconds


def main(args=None):
    rclpy.init(args=args)
    node = FakeBoundaryFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
