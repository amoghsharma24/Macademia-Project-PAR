#!/usr/bin/env python3

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class TrackedTree:
    x: float
    y: float
    observations: int
    last_seen_nanoseconds: int


class TreeMemory(Node):
    def __init__(self):
        super().__init__('tree_memory_node')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('merge_distance', 0.4)
        self.declare_parameter('smoothing_alpha', 0.5)
        self.declare_parameter('min_observations', 1)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('marker_radius', 0.18)
        self.declare_parameter('marker_height', 0.6)
        self.declare_parameter('start_active', True)

        self.active = bool(self.get_parameter('start_active').value)
        self.tracked_trees = []
        self.previous_marker_count = 0

        self.detected_trees_pub = self.create_publisher(PoseArray, '/detected_trees', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/tracked_tree_markers', 10)
        self.status_pub = self.create_publisher(String, '/tree_memory_status', 10)
        self.create_subscription(PoseArray, '/trees', self.trees_callback, 10)
        self.create_subscription(Empty, '/tree_memory_start', self.start_callback, 10)
        self.create_subscription(Empty, '/tree_memory_stop', self.stop_callback, 10)

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate_hz <= 0.0:
            self.get_logger().warn('publish_rate_hz must be positive; using 1.0 Hz')
            publish_rate_hz = 1.0
        self.create_timer(1.0 / publish_rate_hz, self.publish_tracked_trees)

        if self.active:
            self.get_logger().info('Tree memory node started')
            self.publish_status('started')
        else:
            self.get_logger().info('Tree memory node started inactive')
            self.publish_status('stopped')

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def start_callback(self, _msg):
        self.active = True
        self.get_logger().info('Tree memory node started')
        self.publish_status('started')

    def stop_callback(self, _msg):
        self.active = False
        self.get_logger().info('Tree memory node stopped')
        self.publish_status('stopped')

    def trees_callback(self, msg):
        if not self.active:
            return

        merge_distance = max(0.0, float(self.get_parameter('merge_distance').value))
        smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        smoothing_alpha = min(1.0, max(0.0, smoothing_alpha))
        last_seen_nanoseconds = self.get_clock().now().nanoseconds

        for detected_pose in msg.poses:
            detected_x = detected_pose.position.x
            detected_y = detected_pose.position.y
            nearest_tree = None
            nearest_distance = None

            for tracked_tree in self.tracked_trees:
                distance = math.hypot(
                    detected_x - tracked_tree.x,
                    detected_y - tracked_tree.y,
                )
                if nearest_distance is None or distance < nearest_distance:
                    nearest_tree = tracked_tree
                    nearest_distance = distance

            if nearest_tree is not None and nearest_distance <= merge_distance:
                nearest_tree.x = (
                    smoothing_alpha * detected_x
                    + (1.0 - smoothing_alpha) * nearest_tree.x
                )
                nearest_tree.y = (
                    smoothing_alpha * detected_y
                    + (1.0 - smoothing_alpha) * nearest_tree.y
                )
                nearest_tree.observations += 1
                nearest_tree.last_seen_nanoseconds = last_seen_nanoseconds
            else:
                self.tracked_trees.append(
                    TrackedTree(
                        x=detected_x,
                        y=detected_y,
                        observations=1,
                        last_seen_nanoseconds=last_seen_nanoseconds,
                    )
                )

    def publish_tracked_trees(self):
        if not self.active:
            self.publish_status('stopped')
            return

        frame_id = self.get_parameter('frame_id').value
        min_observations = max(
            1,
            int(self.get_parameter('min_observations').value),
        )
        visible_trees = [
            tree
            for tree in self.tracked_trees
            if tree.observations >= min_observations
        ]
        stamp = self.get_clock().now().to_msg()

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = frame_id
        pose_array.poses = [self.create_tree_pose(tree) for tree in visible_trees]
        self.detected_trees_pub.publish(pose_array)

        self.marker_pub.publish(
            self.create_tree_markers(visible_trees, frame_id, stamp)
        )

    def create_tree_pose(self, tracked_tree):
        pose = Pose()
        pose.position.x = tracked_tree.x
        pose.position.y = tracked_tree.y
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def create_tree_markers(self, visible_trees, frame_id, stamp):
        marker_array = MarkerArray()
        radius = max(0.0, float(self.get_parameter('marker_radius').value))
        height = max(0.0, float(self.get_parameter('marker_height').value))

        for marker_id, tracked_tree in enumerate(visible_trees):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'tracked_trees'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = tracked_tree.x
            marker.pose.position.y = tracked_tree.y
            marker.pose.position.z = height * 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = height
            marker.color.r = 0.0
            marker.color.g = 0.75
            marker.color.b = 0.85
            marker.color.a = 0.9
            marker_array.markers.append(marker)

        for marker_id in range(len(visible_trees), self.previous_marker_count):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'tracked_trees'
            marker.id = marker_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.previous_marker_count = len(visible_trees)
        return marker_array


def main(args=None):
    rclpy.init(args=args)
    node = TreeMemory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
