#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int32, String


class MissionState(Enum):
    WAITING_TO_START = 'waiting_to_start'
    DETECTING_TREES = 'detecting_trees'
    PLANNING_TREE_WAYPOINTS = 'planning_tree_waypoints'
    NAVIGATING_TO_TREE = 'navigating_to_tree'
    RUNNING_TREE_BEHAVIOUR = 'running_tree_behaviour'
    MOVING_TO_NEXT_TREE = 'moving_to_next_tree'
    RETURNING_HOME = 'returning_home'
    COMPLETE = 'complete'
    FAILED = 'failed'


class OrchardControlNode(Node):
    def __init__(self):
        super().__init__('orchard_control_node')

        self.declare_parameter('auto_start', False)
        self.declare_parameter('state_period_sec', 0.5)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('nav2_action_name', '/navigate_to_pose')
        self.declare_parameter('nav2_timeout_sec', 5.0)
        self.declare_parameter('spiral_min_radius', 0.25)
        self.declare_parameter('spiral_max_radius', 0.0)
        self.declare_parameter('spiral_loop_spacing', 1.0)

        self.state = MissionState.WAITING_TO_START
        self.mission_active = bool(self.get_parameter('auto_start').value)
        self.detected_trees = []
        self.selected_waypoint = None
        self.selected_tree_index = None
        self.visited_tree_indices = set()
        self.planner_reported_all_visited = False
        self.tree_navigation_reached = False
        self.tree_behaviour_started = False
        self.tree_behaviour_done = False
        self.return_goal_active = False
        self.return_goal_done = False
        self.return_goal_succeeded = False

        self.state_pub = self.create_publisher(String, '/orchard_controller/state', 10)
        self.tree_generator_start_pub = self.create_publisher(Empty, '/tree_generator_start', 10)
        self.tree_memory_start_pub = self.create_publisher(Empty, '/tree_memory_start', 10)
        self.tree_waypoint_start_pub = self.create_publisher(Empty, '/tree_waypoint_start', 10)
        self.tree_waypoint_stop_pub = self.create_publisher(Empty, '/tree_waypoint_stop', 10)
        self.nav2_sender_start_pub = self.create_publisher(Empty, '/nav2_sender_start', 10)
        self.nav2_sender_stop_pub = self.create_publisher(Empty, '/nav2_sender_stop', 10)
        self.reset_visited_pub = self.create_publisher(Empty, '/reset_visited_trees', 10)
        self.mark_tree_visited_pub = self.create_publisher(Int32, '/mark_tree_visited', 10)
        self.start_spiral_pub = self.create_publisher(Float32MultiArray, '/start_spiral', 10)
        self.stop_spiral_pub = self.create_publisher(Empty, '/stop_spiral', 10)

        self.return_home_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter('nav2_action_name').value,
        )

        self.create_subscription(Empty, '/orchard_controller/start', self.start_callback, 10)
        self.create_subscription(PoseArray, '/detected_trees', self.detected_trees_callback, 10)
        self.create_subscription(
            PoseStamped,
            '/selected_tree_waypoint',
            self.selected_waypoint_callback,
            10,
        )
        self.create_subscription(
            String,
            '/tree_waypoint_status',
            self.tree_waypoint_status_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/reached_tree_waypoint',
            self.reached_tree_waypoint_callback,
            10,
        )
        self.create_subscription(Empty, '/spiral_done', self.spiral_done_callback, 10)

        period = float(self.get_parameter('state_period_sec').value)
        self.create_timer(period, self.control_loop)

        self.get_logger().info('Orchard control state machine started')
        self.publish_state()

    def start_callback(self, _msg):
        self.start_mission()

    def detected_trees_callback(self, msg):
        self.detected_trees = list(msg.poses)

    def selected_waypoint_callback(self, msg):
        self.selected_waypoint = msg

    def tree_waypoint_status_callback(self, msg):
        status = msg.data
        if status == 'all_waypoints_visited':
            self.planner_reported_all_visited = True
            return

        prefix = 'selected_waypoint_'
        if not status.startswith(prefix):
            return

        try:
            self.selected_tree_index = int(status[len(prefix):])
        except ValueError:
            self.get_logger().warn(f'Ignoring unrecognised tree waypoint status: {status}')

    def reached_tree_waypoint_callback(self, msg):
        if msg.data and self.state == MissionState.NAVIGATING_TO_TREE:
            self.tree_navigation_reached = True

    def spiral_done_callback(self, _msg):
        if self.state == MissionState.RUNNING_TREE_BEHAVIOUR:
            self.tree_behaviour_done = True

    def control_loop(self):
        self.publish_state()

        if not self.mission_active:
            return

        if self.state == MissionState.WAITING_TO_START:
            self.change_state(MissionState.DETECTING_TREES)
            return

        if self.state == MissionState.DETECTING_TREES:
            if self.detect_trees():
                self.change_state(MissionState.PLANNING_TREE_WAYPOINTS)
            return

        if self.state == MissionState.PLANNING_TREE_WAYPOINTS:
            if self.plan_tree_waypoints():
                self.change_state(MissionState.NAVIGATING_TO_TREE)
            return

        if self.state == MissionState.NAVIGATING_TO_TREE:
            if self.navigate_to_current_tree():
                self.change_state(MissionState.RUNNING_TREE_BEHAVIOUR)
            return

        if self.state == MissionState.RUNNING_TREE_BEHAVIOUR:
            if self.run_tree_behaviour():
                self.change_state(MissionState.MOVING_TO_NEXT_TREE)
            return

        if self.state == MissionState.MOVING_TO_NEXT_TREE:
            self.mark_current_tree_visited()
            self.change_state(MissionState.PLANNING_TREE_WAYPOINTS)
            return

        if self.state == MissionState.RETURNING_HOME:
            if self.return_home():
                self.change_state(MissionState.COMPLETE)

    def start_mission(self):
        self.mission_active = True
        self.detected_trees = []
        self.selected_waypoint = None
        self.selected_tree_index = None
        self.visited_tree_indices.clear()
        self.planner_reported_all_visited = False
        self.tree_navigation_reached = False
        self.tree_behaviour_started = False
        self.tree_behaviour_done = False
        self.return_goal_active = False
        self.return_goal_done = False
        self.return_goal_succeeded = False

        self.publish_empty(self.reset_visited_pub)
        self.publish_empty(self.tree_generator_start_pub)
        self.publish_empty(self.tree_memory_start_pub)
        self.publish_empty(self.tree_waypoint_start_pub)
        self.publish_empty(self.nav2_sender_stop_pub)
        self.publish_empty(self.stop_spiral_pub)
        self.change_state(MissionState.DETECTING_TREES)

    def detect_trees(self):
        self.publish_empty(self.tree_generator_start_pub)
        self.publish_empty(self.tree_memory_start_pub)
        self.publish_empty(self.tree_waypoint_start_pub)

        if self.detected_trees:
            return True

        self.get_logger().info(
            'Waiting for tree memory or detection output on /detected_trees',
            throttle_duration_sec=2.0,
        )
        return False

    def plan_tree_waypoints(self):
        self.publish_empty(self.tree_waypoint_start_pub)
        self.publish_empty(self.nav2_sender_stop_pub)

        if self.planner_reported_all_visited:
            self.change_state(MissionState.RETURNING_HOME)
            return False

        if self.selected_waypoint is None or self.selected_tree_index is None:
            self.get_logger().info(
                'Waiting for /selected_tree_waypoint and /tree_waypoint_status',
                throttle_duration_sec=2.0,
            )
            return False

        if self.selected_tree_index in self.visited_tree_indices:
            self.get_logger().info(
                f'Waiting for planner to select an unvisited tree '
                f'(last was {self.selected_tree_index})',
                throttle_duration_sec=2.0,
            )
            return False

        return True

    def navigate_to_current_tree(self):
        self.publish_empty(self.tree_waypoint_start_pub)
        self.publish_empty(self.nav2_sender_start_pub)

        if self.tree_navigation_reached:
            self.publish_empty(self.nav2_sender_stop_pub)
            self.tree_navigation_reached = False
            return True

        self.get_logger().info(
            f'Waiting for Nav2 waypoint sender to reach tree {self.selected_tree_index}',
            throttle_duration_sec=2.0,
        )
        return False

    def run_tree_behaviour(self):
        self.publish_empty(self.nav2_sender_stop_pub)

        if not self.tree_behaviour_started:
            self.start_tree_behaviour()
            return False

        if not self.tree_behaviour_done:
            self.get_logger().info(
                'Waiting for spiral behaviour to finish on /spiral_done',
                throttle_duration_sec=2.0,
            )
            return False

        self.publish_empty(self.stop_spiral_pub)
        return True

    def return_home(self):
        self.publish_empty(self.tree_waypoint_stop_pub)
        self.publish_empty(self.nav2_sender_stop_pub)
        self.publish_empty(self.stop_spiral_pub)

        if self.return_goal_done:
            if self.return_goal_succeeded:
                self.clear_return_goal_state()
                return True

            self.clear_return_goal_state()
            self.fail('Return home navigation failed')
            return False

        if self.return_goal_active:
            return False

        return self.send_return_home_goal()

    def start_tree_behaviour(self):
        tree_pose = self.current_tree_pose()
        if tree_pose is None:
            self.fail(f'No detected tree pose available for tree {self.selected_tree_index}')
            return

        min_radius = float(self.get_parameter('spiral_min_radius').value)
        max_radius = self.spiral_max_radius_for_tree(tree_pose.position, min_radius)

        msg = Float32MultiArray()
        msg.data = [
            float(tree_pose.position.x),
            float(tree_pose.position.y),
            min_radius,
            max_radius,
            float(self.get_parameter('spiral_loop_spacing').value),
        ]
        self.start_spiral_pub.publish(msg)
        self.tree_behaviour_started = True
        self.tree_behaviour_done = False
        self.get_logger().info(
            f'Started tree behaviour for tree {self.selected_tree_index} '
            f'with max_radius={max_radius:.3f}'
        )

    def spiral_max_radius_for_tree(self, tree_position, min_radius):
        spacing_adjustment = float(self.get_parameter('spiral_max_radius').value)
        neighbour_distances = self.tree_neighbour_distances(tree_position)

        if not neighbour_distances:
            fallback_radius = max(min_radius + 0.01, abs(spacing_adjustment))
            self.get_logger().warn(
                f'Cannot calculate spiral max radius from tree spacing for tree '
                f'{self.selected_tree_index}; using fallback max_radius={fallback_radius:.3f}'
            )
            return fallback_radius

        average_spacing = sum(neighbour_distances) / len(neighbour_distances)
        max_radius = (average_spacing + spacing_adjustment) * 0.5

        if max_radius <= min_radius:
            self.get_logger().warn(
                f'Calculated spiral max radius {max_radius:.3f} is not greater than '
                f'min_radius {min_radius:.3f}; clamping to min_radius + 0.01'
            )
            max_radius = min_radius + 0.01

        self.get_logger().info(
            f'Calculated spiral max radius from {len(neighbour_distances)} neighbouring '
            f'tree distance(s): average_spacing={average_spacing:.3f}, '
            f'spacing_adjustment={spacing_adjustment:.3f}, max_radius={max_radius:.3f}'
        )
        return max_radius

    def tree_neighbour_distances(self, tree_position):
        distances = []
        for index, pose in enumerate(self.detected_trees):
            if index == self.selected_tree_index:
                continue

            dx = pose.position.x - tree_position.x
            dy = pose.position.y - tree_position.y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            if distance > 0.0:
                distances.append(distance)

        return sorted(distances)[:2]

    def current_tree_pose(self):
        if self.selected_tree_index is None:
            return None

        if self.selected_tree_index < 0 or self.selected_tree_index >= len(self.detected_trees):
            return None

        return self.detected_trees[self.selected_tree_index]

    def mark_current_tree_visited(self):
        if self.selected_tree_index is None:
            return

        msg = Int32()
        msg.data = int(self.selected_tree_index)
        self.mark_tree_visited_pub.publish(msg)
        self.visited_tree_indices.add(self.selected_tree_index)
        self.get_logger().info(f'Marked tree {self.selected_tree_index} visited')

    def send_return_home_goal(self):
        timeout = float(self.get_parameter('nav2_timeout_sec').value)
        if not self.return_home_client.wait_for_server(timeout_sec=timeout):
            self.fail('Nav2 action server is not available for return home')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self.home_pose()

        self.get_logger().info('Sending return-home Nav2 goal')
        future = self.return_home_client.send_goal_async(goal)
        future.add_done_callback(self.return_goal_response_callback)
        self.return_goal_active = True
        self.return_goal_done = False
        self.return_goal_succeeded = False
        return False

    def return_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected return-home goal')
            self.return_goal_done = True
            self.return_goal_succeeded = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.return_goal_result_callback)

    def return_goal_result_callback(self, future):
        result = future.result()
        self.return_goal_done = True
        self.return_goal_succeeded = result.status == GoalStatus.STATUS_SUCCEEDED

    def clear_return_goal_state(self):
        self.return_goal_active = False
        self.return_goal_done = False
        self.return_goal_succeeded = False

    def home_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter('frame_id').value
        pose.pose.position.x = float(self.get_parameter('home_x').value)
        pose.pose.position.y = float(self.get_parameter('home_y').value)
        pose.pose.position.z = 0.0

        yaw = float(self.get_parameter('home_yaw').value)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def change_state(self, new_state):
        if self.state == new_state:
            return

        old_state = self.state
        self.state = new_state

        if new_state == MissionState.PLANNING_TREE_WAYPOINTS:
            self.selected_waypoint = None
            self.selected_tree_index = None
            self.tree_navigation_reached = False
        elif new_state == MissionState.NAVIGATING_TO_TREE:
            self.tree_navigation_reached = False
        elif new_state == MissionState.RUNNING_TREE_BEHAVIOUR:
            self.tree_behaviour_started = False
            self.tree_behaviour_done = False
        elif new_state == MissionState.COMPLETE:
            self.mission_active = False

        self.get_logger().info(f'State changed: {old_state.value} -> {new_state.value}')
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def publish_empty(self, publisher):
        publisher.publish(Empty())

    def fail(self, reason):
        self.get_logger().error(reason)
        self.publish_empty(self.nav2_sender_stop_pub)
        self.publish_empty(self.stop_spiral_pub)
        self.change_state(MissionState.FAILED)


def main(args=None):
    rclpy.init(args=args)
    node = OrchardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
