#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node


class FakeTreePublisher(Node):
    def __init__(self):
        super().__init__('fake_tree_publisher')

        self.declare_parameter('frame_id', 'map')
        self.publisher = self.create_publisher(PoseArray, '/detected_trees', 10)
        self.create_timer(1.0, self.publish_fake_trees)

        self.get_logger().info('Fake tree publisher started')

    def publish_fake_trees(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.get_parameter('frame_id').value
        pose_array.poses = self.create_fake_tree_poses()

        self.publisher.publish(pose_array)

    def create_fake_tree_poses(self):
        tree_positions = [
            (0.5, 1.0),
            (1.5, 1.0),
            (2.5, 1.0),
            (3.5, 1.0),
            (0.5, -1.0),
            (1.5, -1.0),
            (2.5, -1.0),
            (3.5, -1.0),
        ]

        tree_poses = []
        for x, y in tree_positions:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            tree_poses.append(pose)

        return tree_poses


def main(args=None):
    rclpy.init(args=args)
    node = FakeTreePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
