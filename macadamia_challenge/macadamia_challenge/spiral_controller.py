import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty


class SpiralController(Node):
    def __init__(self):
        super().__init__('spiral_controller')

        self.declare_parameter('center_x', 0.5)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('min_radius', 0.1)
        self.declare_parameter('max_radius', 4.0)
        self.declare_parameter('loop_spacing', 0.3)
        self.declare_parameter('linear_speed', 0.125)
        self.declare_parameter('kp_heading', 1.5)

        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.min_radius = self.get_parameter('min_radius').value
        self.max_radius = self.get_parameter('max_radius').value
        self.loop_spacing = self.get_parameter('loop_spacing').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/spiral_markers', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.started = False

        self.start_sub = self.create_subscription(
            Empty,
            '/trigger_start',
            self.start_callback,
            10
        )

        self.get_logger().info(
            f'Spiral centre set to: x={self.center_x:.3f}, y={self.center_y:.3f}'
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

        self.k = self.loop_spacing / (2.0 * math.pi)

        self.theta = 0.0
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)

    def control_loop(self):

        radius = self.min_radius + self.k * self.theta

        if radius > self.max_radius:
            self.stop_robot()
            self.get_logger().info('Finished spiral path')
            return

        target_x = self.center_x + radius * math.cos(self.theta)
        target_y = self.center_y + radius * math.sin(self.theta)

        self.get_logger().info(
            f'Target point: x={target_x:.3f}, y={target_y:.3f}, '
            f'radius={radius:.3f}, theta={self.theta:.3f}'
        )

        self.publish_marker(0, self.center_x, self.center_y, 0.0, 1.0, 0.0, 'centre_point')
        self.publish_marker(1, target_x, target_y, 1.0, 0.0, 0.0, 'target_point')

        if not self.started:
            self.stop_robot()
            return

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

        self.get_logger().info(
            f'Pose: x={self.current_x:.3f}, y={self.current_y:.3f}, yaw={self.current_yaw:.3f} | '
            f'Heading error={heading_error:.3f}, distance={distance_error:.3f}, '
            f'linear={cmd.twist.linear.x:.3f}, angular={cmd.twist.angular.z:.3f}'
        )

        self.cmd_pub.publish(cmd)

        if distance_error < 0.15:
            self.theta += 0.04

    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self.cmd_pub.publish(cmd)

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

        self.started = True
        self.theta = 0.0

        self.get_logger().info('Spiral movement started')
        self.get_logger().info(
            f'Start trigger received at robot pose: '
            f'x={self.current_x:.3f}, y={self.current_y:.3f}, yaw={self.current_yaw:.3f}'
        )

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