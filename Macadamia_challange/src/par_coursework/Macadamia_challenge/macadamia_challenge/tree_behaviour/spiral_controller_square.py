import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty, Float32MultiArray


class SpiralController(Node):
    def __init__(self):
        super().__init__('spiral_controller_square')

        self.declare_parameter('center_x', 0.5)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('min_radius', 0.1)
        self.declare_parameter('max_radius', 4.0)
        self.declare_parameter('loop_spacing', 1.0)
        self.declare_parameter('robot_width', 0.233)
        self.declare_parameter('linear_speed', 0.125)
        self.declare_parameter('kp_heading', 1.5)

        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.min_radius = self.get_parameter('min_radius').value
        self.max_radius = self.get_parameter('max_radius').value
        self.loop_spacing = self.get_parameter('loop_spacing').value
        self.robot_width = self.get_parameter('robot_width').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value

        self.current_loop = 0
        self.current_corner = 0
        self.current_target = None

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/spiral_markers', 10)
        self.done_pub = self.create_publisher(Empty, '/spiral_done', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.started = False
        self.finished = False

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

        self.get_logger().info(
            f'Square spiral centre set to: x={self.center_x:.3f}, y={self.center_y:.3f}'
        )

        self.publish_marker(
            marker_id=0,
            x=self.center_x,
            y=self.center_y,
            r=0.0,
            g=1.0,
            b=0.0,
            name='centre_point'
        )

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)

    def control_loop(self):
        if not self.started:
            return

        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            self.get_logger().warn('Waiting for odometry before running spiral', throttle_duration_sec=2.0)
            return

        if self.current_target is None:
            if not self.set_next_target():
                self.finish_spiral()
                return

        target_x, target_y = self.current_target
        target_yaw = self.corner_yaw(self.current_corner)

        self.publish_marker(0, self.center_x, self.center_y, 0.0, 1.0, 0.0, 'centre_point')
        self.publish_marker(1, target_x, target_y, 1.0, 0.0, 0.0, 'target_point')

        dx = target_x - self.current_x
        dy = target_y - self.current_y

        distance_error = math.sqrt(dx ** 2 + dy ** 2)
        target_heading = math.atan2(dy, dx)
        heading_error = self.angle_normalise(target_heading - self.current_yaw)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if abs(heading_error) > 0.25:
            cmd.twist.linear.x = 0.0
        else:
            cmd.twist.linear.x = self.linear_speed

        cmd.twist.angular.z = self.kp_heading * heading_error
        cmd.twist.angular.z = max(min(cmd.twist.angular.z, 0.4), -0.4)

        self.cmd_pub.publish(cmd)

        if distance_error < 0.15:
            self.advance_target()

    def set_next_target(self):
        radius = self.radius_for_loop(self.current_loop)
        if radius > self.max_radius:
            return False

        self.current_target = self.corner_point(self.current_loop, self.current_corner)
        return True

    def advance_target(self):
        self.current_corner += 1
        if self.current_corner >= 4:
            self.current_corner = 0
            self.current_loop += 1

        if self.radius_for_loop(self.current_loop) > self.max_radius:
            self.finish_spiral()
            return

        self.current_target = self.corner_point(self.current_loop, self.current_corner)

    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self.cmd_pub.publish(cmd)

    def finish_spiral(self):
        if self.finished:
            return

        self.finished = True
        self.started = False
        self.stop_robot()
        self.done_pub.publish(Empty())
        self.get_logger().info('Finished square spiral path')

    def publish_marker(self, marker_id, x, y, r, g, b, name):
        marker = Marker()
        marker.header.frame_id = 'odom'
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

        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.min_radius = float(min_radius)
        self.max_radius = float(max_radius)
        self.loop_spacing = float(loop_spacing)

        self.current_loop = 0
        self.current_corner = 0
        self.current_target = None
        self.started = True
        self.finished = False

        self.get_logger().info('Square spiral movement started')
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
        was_started = self.started

        self.started = False
        self.finished = False

        if was_started:
            self.get_logger().info('Square spiral movement stopped')
            self.stop_robot()

    def loop_spacing_metres(self):
        return self.loop_spacing * self.robot_width

    def radius_for_loop(self, loop_index):
        return self.min_radius + loop_index * self.loop_spacing_metres()

    def corner_point(self, loop_index, corner_index):
        radius = self.radius_for_loop(loop_index)
        if corner_index == 0:
            return self.center_x + radius, self.center_y + radius
        if corner_index == 1:
            return self.center_x - radius, self.center_y + radius
        if corner_index == 2:
            return self.center_x - radius, self.center_y - radius
        return self.center_x + radius, self.center_y - radius

    @staticmethod
    def corner_yaw(corner_index):
        if corner_index == 0:
            return math.pi
        if corner_index == 1:
            return -math.pi / 2.0
        if corner_index == 2:
            return 0.0
        return math.pi / 2.0

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def angle_normalise(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = SpiralController()
    rclpy.spin(node)
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
