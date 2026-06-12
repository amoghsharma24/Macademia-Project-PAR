import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, Float32MultiArray
from visualization_msgs.msg import Marker


class SpiralController(Node):
    def __init__(self):
        super().__init__('spiral_controller')

        self.declare_parameter('center_x', 0.5)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('min_radius', 0.1)
        self.declare_parameter('max_radius', 4.0)
        self.declare_parameter('loop_spacing', 1.0)
        self.declare_parameter('robot_width', 0.233)
        self.declare_parameter('theta_step', 0.04)
        self.declare_parameter('waypoint_spacing', 0.35)
        self.declare_parameter('max_theta_step', 0.45)
        self.declare_parameter('batch_size', 8)
        self.declare_parameter('start_with_min_radius_circle', True)
        self.declare_parameter('initial_circle_angle', 2.0 * math.pi)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('stamp_goals', False)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('nav2_action_name', 'navigate_through_poses')
        self.declare_parameter('yaw_mode', 'path')
        self.declare_parameter('max_fallback_attempts', 4)
        self.declare_parameter('max_consecutive_rejections', 6)
        self.declare_parameter('fallback_angle_step', 0.35)
        self.declare_parameter('fallback_radius_step', 0.15)

        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.min_radius = self.get_parameter('min_radius').value
        self.max_radius = self.get_parameter('max_radius').value
        self.loop_spacing = self.get_parameter('loop_spacing').value
        self.robot_width = self.get_parameter('robot_width').value
        self.theta_step = self.get_parameter('theta_step').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.max_theta_step = self.get_parameter('max_theta_step').value
        self.batch_size = self.get_parameter('batch_size').value
        self.start_with_min_radius_circle = self.get_parameter('start_with_min_radius_circle').value
        self.initial_circle_angle = self.get_parameter('initial_circle_angle').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.stamp_goals = self.get_parameter('stamp_goals').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.nav2_action_name = self.get_parameter('nav2_action_name').value
        self.yaw_mode = self.get_parameter('yaw_mode').value
        self.max_fallback_attempts = self.get_parameter('max_fallback_attempts').value
        self.max_consecutive_rejections = self.get_parameter('max_consecutive_rejections').value
        self.fallback_angle_step = self.get_parameter('fallback_angle_step').value
        self.fallback_radius_step = self.get_parameter('fallback_radius_step').value

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/spiral_markers', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.start_sub = self.create_subscription(
            Float32MultiArray,
            '/start_spiral',
            self.start_callback,
            10
        )
        self.stop_sub = self.create_subscription(
            Empty,
            '/stop_spiral',
            self.stop_callback,
            10
        )

        self.nav_client = ActionClient(self, NavigateThroughPoses, self.nav2_action_name)

        self.started = False
        self.finished = False
        self.goal_active = False
        self.goal_handle = None
        self.current_goal_is_fallback = False
        self.current_spiral_goal = None
        self.current_batch = []
        self.fallback_attempts = 0
        self.consecutive_rejections = 0

        self.k = self.spiral_growth_rate()
        if not self.start_with_min_radius_circle:
            self.initial_circle_angle = 0.0
        self.theta = 0.0
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.get_logger().info(
            f'Spiral centre set to: x={self.center_x:.3f}, y={self.center_y:.3f}'
        )
        self.get_logger().info(f'Waiting for odometry on: {self.odom_topic}')
        self.publish_marker(
            marker_id=0,
            x=self.center_x,
            y=self.center_y,
            r=0.0,
            g=1.0,
            b=0.0,
            name='centre_point'
        )

        self.timer = self.create_timer(0.2, self.control_loop)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)

    def control_loop(self):
        self.publish_marker(0, self.center_x, self.center_y, 0.0, 1.0, 0.0, 'centre_point')

        if not self.started:
            return

        if self.goal_active:
            return

        radius = self.radius_for_theta(self.theta)
        if radius > self.max_radius:
            self.finish_spiral()
            return

        self.send_spiral_batch()

    def send_spiral_batch(self):
        self.current_batch = self.build_spiral_batch()
        if not self.current_batch:
            self.finish_spiral()
            return

        x, y, yaw, radius, theta = self.current_batch[0]
        self.current_spiral_goal = (x, y, yaw, radius, theta)
        self.fallback_attempts = 0
        self.publish_marker(1, x, y, 1.0, 0.0, 0.0, 'spiral_goal')
        poses = [
            self.make_pose_stamped(goal_x, goal_y, goal_yaw)
            for goal_x, goal_y, goal_yaw, _, _ in self.current_batch
        ]
        self.send_nav_goal(poses, is_fallback=False)

    def build_spiral_batch(self):
        batch = []
        theta = self.theta

        while len(batch) < max(1, int(self.batch_size)):
            radius = self.radius_for_theta(theta)
            if radius > self.max_radius:
                break

            x, y, yaw, radius = self.spiral_pose_for_theta(theta)
            batch.append((x, y, yaw, radius, theta))
            theta += self.theta_step_for_radius(radius)

        return batch

    def send_fallback_goal(self):
        if self.current_spiral_goal is None:
            self.advance_spiral()
            return

        if self.fallback_attempts >= self.max_fallback_attempts:
            x, y, _, _, theta = self.current_spiral_goal
            self.get_logger().warn(
                f'Skipping unreachable spiral step at theta={theta:.3f}, '
                f'x={x:.3f}, y={y:.3f} after {self.fallback_attempts} fallback attempts'
            )
            self.advance_spiral()
            return

        self.fallback_attempts += 1
        x, y, yaw = self.fallback_pose(self.fallback_attempts)
        self.publish_marker(2, x, y, 1.0, 0.8, 0.0, 'fallback_goal')
        self.get_logger().info(
            f'Sending fallback goal {self.fallback_attempts}/{self.max_fallback_attempts}: '
            f'x={x:.3f}, y={y:.3f}'
        )
        self.send_nav_goal([self.make_pose_stamped(x, y, yaw)], is_fallback=True)

    def send_nav_goal(self, poses, is_fallback):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.goal_active = True
        self.current_goal_is_fallback = is_fallback

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        goal_name = 'fallback' if is_fallback else 'spiral'
        first_pose = poses[0].pose
        last_pose = poses[-1].pose
        self.get_logger().info(
            f'Sent {goal_name} Nav2 goal with {len(poses)} pose(s): '
            f'first x={first_pose.position.x:.3f}, y={first_pose.position.y:.3f}; '
            f'last x={last_pose.position.x:.3f}, y={last_pose.position.y:.3f}; '
            f'theta={self.theta:.3f}'
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.goal_active = False
            self.record_goal_rejection()
            if not self.started:
                return

            goal_name = 'fallback' if self.current_goal_is_fallback else 'spiral'
            self.get_logger().warn(f'Nav2 rejected {goal_name} goal')
            self.send_fallback_goal()
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        self.goal_active = False
        self.goal_handle = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.consecutive_rejections = 0
            goal_name = 'fallback' if self.current_goal_is_fallback else 'spiral'
            self.get_logger().info(f'Nav2 reached {goal_name} goal at theta={self.theta:.3f}')
            self.advance_spiral()
            return

        status_name = self.goal_status_name(result.status)
        goal_name = 'fallback' if self.current_goal_is_fallback else 'spiral'
        self.get_logger().warn(f'Nav2 {goal_name} goal ended with status: {status_name}')
        self.send_fallback_goal()

    def advance_spiral(self):
        if self.current_goal_is_fallback or not self.current_batch:
            self.theta += self.theta_step_for_radius(self.radius_for_theta(self.theta))
        else:
            self.theta = self.current_batch[-1][4] + self.theta_step_for_radius(self.current_batch[-1][3])

        self.current_spiral_goal = None
        self.current_batch = []
        self.fallback_attempts = 0

    def record_goal_rejection(self):
        self.consecutive_rejections += 1

        if self.consecutive_rejections < self.max_consecutive_rejections:
            return

        self.get_logger().error(
            'Nav2 is rejecting every goal before planning starts. Stopping spiral '
            'instead of skipping through waypoints.'
        )
        self.get_logger().error(
            f'Check Nav2 lifecycle state, goal_frame, /start_spiral coordinates, '
            f'use_sim_time, and whether the first goal is inside an inflated obstacle. '
            f'Last state: theta={self.theta:.3f}, centre=({self.center_x:.3f}, '
            f'{self.center_y:.3f}), goal_frame={self.goal_frame}, '
            f'stamp_goals={self.stamp_goals}, action={self.nav2_action_name}'
        )
        self.finish_spiral()

    def finish_spiral(self):
        if self.finished:
            return

        self.finished = True
        self.started = False
        self.cancel_active_goal()
        self.stop_robot()
        self.get_logger().info('Finished spiral path')

    def cancel_active_goal(self):
        if self.goal_handle is None:
            self.goal_active = False
            return

        self.goal_handle.cancel_goal_async()
        self.goal_active = False
        self.goal_handle = None

    def fallback_pose(self, attempt):
        _, _, _, radius, theta = self.current_spiral_goal

        side = 1.0 if attempt % 2 == 1 else -1.0
        angle_offset = side * math.ceil(attempt / 2.0) * self.fallback_angle_step
        radius_offset = self.fallback_radius_step * math.floor((attempt - 1) / 2.0)

        fallback_theta = theta + angle_offset
        fallback_radius = min(radius + radius_offset, self.max_radius)
        x = self.center_x + fallback_radius * math.cos(fallback_theta)
        y = self.center_y + fallback_radius * math.sin(fallback_theta)
        yaw = self.spiral_tangent_yaw(fallback_theta, fallback_radius)
        return x, y, yaw

    def spiral_pose_for_theta(self, theta):
        radius = self.radius_for_theta(theta)
        x = self.center_x + radius * math.cos(theta)
        y = self.center_y + radius * math.sin(theta)
        yaw = self.spiral_tangent_yaw(theta, radius)
        return x, y, yaw, radius

    def radius_for_theta(self, theta):
        spiral_theta = max(theta - self.initial_circle_angle, 0.0)
        return self.min_radius + self.k * spiral_theta

    def theta_step_for_radius(self, radius):
        radius = max(radius, 0.001)
        spacing_step = self.waypoint_spacing / radius
        return min(max(self.theta_step, spacing_step), self.max_theta_step)

    def spiral_tangent_yaw(self, theta, radius):
        if self.yaw_mode == 'none':
            return self.current_yaw if self.current_yaw is not None else 0.0

        if theta < self.initial_circle_angle:
            return self.angle_normalise(theta + math.pi / 2.0)

        dx_dtheta = self.k * math.cos(theta) - radius * math.sin(theta)
        dy_dtheta = self.k * math.sin(theta) + radius * math.cos(theta)
        return math.atan2(dy_dtheta, dx_dtheta)

    def make_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        if self.stamp_goals:
            pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion_z_w(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self.cmd_pub.publish(cmd)

    def publish_marker(self, marker_id, x, y, r, g, b, name):
        marker = Marker()
        marker.header.frame_id = self.goal_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def start_callback(self, msg):
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn('Start trigger ignored: odometry not available yet')
            return

        if len(msg.data) != 5:
            self.get_logger().warn(
                'Start trigger ignored: expected Float32MultiArray data '
                '[center_x, center_y, min_radius, max_radius, loop_spacing_widths]'
            )
            return

        center_x, center_y, min_radius, max_radius, loop_spacing = msg.data

        if min_radius < 0.0:
            self.get_logger().warn('Start trigger ignored: min_radius must be >= 0.0')
            return

        if max_radius <= min_radius:
            self.get_logger().warn('Start trigger ignored: max_radius must be greater than min_radius')
            return

        if loop_spacing <= 0.0:
            self.get_logger().warn('Start trigger ignored: loop_spacing_widths must be > 0.0')
            return

        if self.goal_frame == 'map':
            self.get_logger().warn(
                'Spiral goals are being sent in the map frame. Make sure /start_spiral '
                'centre coordinates are also in map, not odom.'
            )

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f'Start trigger ignored: Nav2 action server not available: {self.nav2_action_name}'
            )
            return

        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.min_radius = float(min_radius)
        self.max_radius = float(max_radius)
        self.loop_spacing = float(loop_spacing)
        self.k = self.spiral_growth_rate()

        self.cancel_active_goal()
        self.started = True
        self.finished = False
        self.theta = 0.0
        self.current_spiral_goal = None
        self.current_batch = []
        self.fallback_attempts = 0
        self.consecutive_rejections = 0

        self.get_logger().info('Spiral Nav2 waypoint movement started')
        self.get_logger().info(
            f'Spiral settings from start message: centre x={self.center_x:.3f}, '
            f'y={self.center_y:.3f}, min_radius={self.min_radius:.3f}, '
            f'max_radius={self.max_radius:.3f}, loop_spacing={self.loop_spacing:.3f} '
            f'robot widths ({self.loop_spacing_metres():.3f} m)'
        )
        self.get_logger().info(
            f'Start trigger received at robot pose: '
            f'x={self.current_x:.3f}, y={self.current_y:.3f}, yaw={self.current_yaw:.3f}'
        )

    def stop_callback(self, _msg):
        was_started = self.started or self.goal_active

        self.started = False
        self.finished = False
        self.cancel_active_goal()

        if was_started:
            self.get_logger().info('Spiral Nav2 waypoint movement stopped')
            self.stop_robot()

    def loop_spacing_metres(self):
        return self.loop_spacing * self.robot_width

    def spiral_growth_rate(self):
        return self.loop_spacing_metres() / (2.0 * math.pi)

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion_z_w(yaw):
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    @staticmethod
    def angle_normalise(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def goal_status_name(status):
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, f'UNRECOGNISED_{status}')


def main(args=None):
    rclpy.init(args=args)
    node = SpiralController()
    rclpy.spin(node)
    node.cancel_active_goal()
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
