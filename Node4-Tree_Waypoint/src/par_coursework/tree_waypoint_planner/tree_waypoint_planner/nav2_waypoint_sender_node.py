#!/usr/bin/env python3

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Nav2WaypointSender(Node):
    def __init__(self):
        super().__init__('nav2_waypoint_sender')

        self.declare_parameter('auto_send', False)
        self.declare_parameter('send_once', True)
        self.declare_parameter('duplicate_distance_tolerance', 0.05)
        self.declare_parameter('duplicate_yaw_tolerance', 0.1)
        self.declare_parameter('action_server_timeout_sec', 5.0)

        self.latest_waypoint = None
        self.last_sent_waypoint = None

        self.status_pub = self.create_publisher(String, '/nav2_waypoint_status', 10)
        self.reached_pub = self.create_publisher(Bool, '/reached_tree_waypoint', 10)
        self.create_subscription(
            PoseStamped,
            '/selected_tree_waypoint',
            self.selected_waypoint_callback,
            10,
        )

        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.publish_status('waiting_for_waypoint')
        self.get_logger().info('Nav2 waypoint sender started (auto_send default: false)')

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def selected_waypoint_callback(self, msg):
        self.latest_waypoint = msg
        self.get_logger().info(
            f'Received selected waypoint: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}'
        )
        self.publish_status('received_waypoint')

        auto_send = self.get_parameter('auto_send').value
        if not auto_send:
            return

        if self.is_duplicate_waypoint(msg):
            self.get_logger().info('Duplicate waypoint ignored')
            self.publish_status('duplicate_ignored')
            return

        self.send_goal(msg)

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def is_duplicate_waypoint(self, waypoint):
        if self.last_sent_waypoint is None:
            return False

        send_once = self.get_parameter('send_once').value
        if not send_once:
            return False

        distance_tol = self.get_parameter('duplicate_distance_tolerance').value
        yaw_tol = self.get_parameter('duplicate_yaw_tolerance').value

        dx = waypoint.pose.position.x - self.last_sent_waypoint.pose.position.x
        dy = waypoint.pose.position.y - self.last_sent_waypoint.pose.position.y
        distance = math.hypot(dx, dy)

        yaw_new = self.yaw_from_quaternion(waypoint.pose.orientation)
        yaw_old = self.yaw_from_quaternion(self.last_sent_waypoint.pose.orientation)
        yaw_diff = abs(self.normalize_angle(yaw_new - yaw_old))

        return distance <= distance_tol and yaw_diff <= yaw_tol

    def send_goal(self, waypoint):
        timeout_sec = self.get_parameter('action_server_timeout_sec').value
        if not self.nav2_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().warn('Nav2 action server unavailable')
            self.publish_status('waiting_for_nav2')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint

        self.publish_status('sent_goal')
        self.get_logger().info('Sending goal to Nav2 /navigate_to_pose')

        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.last_sent_waypoint = waypoint

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            self.publish_status('goal_rejected')
            return

        self.get_logger().info('Nav2 goal accepted')
        self.publish_status('goal_accepted')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 goal succeeded')
            self.publish_status('goal_succeeded')
            reached = Bool()
            reached.data = True
            self.reached_pub.publish(reached)
            return

        self.get_logger().warn(f'Nav2 goal failed with status code: {result.status}')
        self.publish_status('goal_failed')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2WaypointSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
