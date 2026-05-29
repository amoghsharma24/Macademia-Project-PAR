#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class TreeWaypointPlanner(Node):
    def __init__(self):
        super().__init__('tree_waypoint_planner')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('use_hardcoded_trees', True)
        self.declare_parameter('waypoint_mode', 'towards_centerline')
        self.declare_parameter('waypoint_offset_x', 0.0)
        self.declare_parameter('waypoint_offset_y', -0.6)
        self.declare_parameter('centreline_y', 0.0)
        self.declare_parameter('approach_distance', 0.9)
        self.declare_parameter('tree_marker_radius', 0.25)
        self.declare_parameter('tree_marker_height', 0.8)

        self.declare_parameter('selection_mode', 'nearest')
        self.declare_parameter('selected_tree_index', 0)
        self.declare_parameter('reference_x', 0.0)
        self.declare_parameter('reference_y', 0.0)

        self.tree_marker_pub = self.create_publisher(MarkerArray, '/tree_markers', 10)
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, '/tree_waypoint_markers', 10)
        self.waypoint_pub = self.create_publisher(PoseArray, '/tree_waypoints', 10)
        self.selected_waypoint_pub = self.create_publisher(PoseStamped, '/selected_tree_waypoint', 10)
        self.selected_waypoint_marker_pub = self.create_publisher(Marker, '/selected_tree_waypoint_marker', 10)

        self.detected_tree_poses = []
        self.previous_tree_marker_count = 0
        self.previous_waypoint_marker_count = 0
        self.create_subscription(PoseArray, '/detected_trees', self.detected_trees_callback, 10)
        self.create_timer(1.0, self.publish_planner_outputs)

        self.get_logger().info('Tree waypoint planner started')

    def detected_trees_callback(self, msg):
        self.detected_tree_poses = list(msg.poses)

    def publish_planner_outputs(self):
        frame_id = self.get_parameter('frame_id').value
        tree_poses = self.get_tree_poses()
        waypoint_poses = self.create_waypoint_poses(tree_poses)
        stamp = self.get_clock().now().to_msg()

        waypoint_array = PoseArray()
        waypoint_array.header.stamp = stamp
        waypoint_array.header.frame_id = frame_id
        waypoint_array.poses = waypoint_poses

        self.tree_marker_pub.publish(self.create_tree_markers(tree_poses, frame_id, stamp))
        self.waypoint_marker_pub.publish(
            self.create_waypoint_markers(waypoint_poses, frame_id, stamp)
        )
        self.waypoint_pub.publish(waypoint_array)

        selected_index, selected_pose = self.select_waypoint(waypoint_poses)
        if selected_pose is not None:
            selected_waypoint = PoseStamped()
            selected_waypoint.header.stamp = stamp
            selected_waypoint.header.frame_id = frame_id
            selected_waypoint.pose = selected_pose
            self.selected_waypoint_pub.publish(selected_waypoint)
            self.selected_waypoint_marker_pub.publish(
                self.create_selected_waypoint_marker(selected_pose, frame_id, stamp, selected_index)
            )

    def get_tree_poses(self):
        if self.get_parameter('use_hardcoded_trees').value:
            return self.create_hardcoded_tree_poses()

        return self.detected_tree_poses

    def create_hardcoded_tree_poses(self):
        tree_positions = [
            (1.0, 1.2),
            (2.0, 1.2),
            (3.0, 1.2),
            (4.0, 1.2),
            (1.0, -1.2),
            (2.0, -1.2),
            (3.0, -1.2),
            (4.0, -1.2),
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

    def create_waypoint_poses(self, tree_poses):
        waypoint_mode = self.get_parameter('waypoint_mode').value
        offset_x = self.get_parameter('waypoint_offset_x').value
        offset_y = self.get_parameter('waypoint_offset_y').value
        centreline_y = self.get_parameter('centreline_y').value
        approach_distance = self.get_parameter('approach_distance').value
        centreline_margin = 0.05
        waypoint_poses = []

        for tree_pose in tree_poses:
            waypoint_pose = Pose()
            if waypoint_mode == 'fixed_offset':
                waypoint_pose.position.x = tree_pose.position.x + offset_x
                waypoint_pose.position.y = tree_pose.position.y + offset_y
            else:
                waypoint_pose.position.x = tree_pose.position.x
                if tree_pose.position.y > centreline_y:
                    desired_y = tree_pose.position.y - approach_distance
                    max_cross_y = centreline_y + centreline_margin
                    waypoint_pose.position.y = max(desired_y, max_cross_y)
                else:
                    desired_y = tree_pose.position.y + approach_distance
                    min_cross_y = centreline_y - centreline_margin
                    waypoint_pose.position.y = min(desired_y, min_cross_y)
            waypoint_pose.position.z = 0.0

            yaw = math.atan2(
                tree_pose.position.y - waypoint_pose.position.y,
                tree_pose.position.x - waypoint_pose.position.x,
            )
            waypoint_pose.orientation = self.yaw_to_quaternion(yaw)
            waypoint_poses.append(waypoint_pose)

        return waypoint_poses

    def select_waypoint(self, waypoint_poses):
        if not waypoint_poses:
            return None, None

        selection_mode = self.get_parameter('selection_mode').value

        # nearest mode currently uses a fixed reference point (default 0,0),
        # matching the measured demo start pose; later this can use live tf2 robot pose.
        if selection_mode == 'nearest':
            reference_x = self.get_parameter('reference_x').value
            reference_y = self.get_parameter('reference_y').value
            selected_index = 0
            selected_distance = None
            for index, waypoint_pose in enumerate(waypoint_poses):
                dx = waypoint_pose.position.x - reference_x
                dy = waypoint_pose.position.y - reference_y
                distance = math.hypot(dx, dy)
                if selected_distance is None or distance < selected_distance:
                    selected_distance = distance
                    selected_index = index
            return selected_index, waypoint_poses[selected_index]

        # index mode is useful for testing specific waypoints.
        selected_index = int(self.get_parameter('selected_tree_index').value)
        if selected_index < 0:
            selected_index = 0
        if selected_index >= len(waypoint_poses):
            selected_index = len(waypoint_poses) - 1
        return selected_index, waypoint_poses[selected_index]

    def create_selected_waypoint_marker(self, selected_pose, frame_id, stamp, selected_index):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'selected_tree_waypoint'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = selected_pose
        marker.pose.position.z = 0.08
        marker.scale.x = 0.6
        marker.scale.y = 0.14
        marker.scale.z = 0.14
        marker.color.r = 0.1
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = str(selected_index)
        return marker

    def yaw_to_quaternion(self, yaw):
        quaternion = Pose().orientation
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw * 0.5)
        quaternion.w = math.cos(yaw * 0.5)
        return quaternion

    def create_tree_markers(self, tree_poses, frame_id, stamp):
        marker_array = MarkerArray()
        radius = self.get_parameter('tree_marker_radius').value
        height = self.get_parameter('tree_marker_height').value

        for marker_id, tree_pose in enumerate(tree_poses):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'trees'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = tree_pose.position.x
            marker.pose.position.y = tree_pose.position.y
            marker.pose.position.z = height * 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = height
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 0.1
            marker.color.a = 0.8
            marker_array.markers.append(marker)

        self.append_delete_markers(
            marker_array,
            'trees',
            len(tree_poses),
            self.previous_tree_marker_count,
            frame_id,
            stamp,
        )
        self.previous_tree_marker_count = len(tree_poses)

        return marker_array

    def create_waypoint_markers(self, waypoint_poses, frame_id, stamp):
        marker_array = MarkerArray()

        for marker_id, waypoint_pose in enumerate(waypoint_poses):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'tree_waypoints'
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = waypoint_pose
            marker.pose.position.z = 0.05
            marker.scale.x = 0.25
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.r = 1.0
            marker.color.g = 0.45
            marker.color.b = 0.0
            marker.color.a = 0.9
            marker_array.markers.append(marker)

        self.append_delete_markers(
            marker_array,
            'tree_waypoints',
            len(waypoint_poses),
            self.previous_waypoint_marker_count,
            frame_id,
            stamp,
        )
        self.previous_waypoint_marker_count = len(waypoint_poses)

        return marker_array

    def append_delete_markers(self, marker_array, namespace, current_count, previous_count, frame_id, stamp):
        for marker_id in range(current_count, previous_count):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TreeWaypointPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
