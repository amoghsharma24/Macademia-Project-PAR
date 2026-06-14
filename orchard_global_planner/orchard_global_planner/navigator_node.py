import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import NavigateThroughPoses
from std_msgs.msg import String, Bool, Empty
from action_msgs.msg import GoalStatus


class NavigatorNode(Node):
    
    def __init__(self):
        super().__init__('navigator_node')

        self.client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )

        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        self.status_pub = self.create_publisher(String, '/navigator_status', 10)
        self.reached_pub = self.create_publisher(Bool, '/reached_tree_waypoint', 10)


        self.goal_sent = False
    
    def path_callback(self, msg):

        if self.goal_sent:
            return
        if len(msg.poses) == 0:
            self.get_logger().warn("Received emtpy path")
            return
        
        self.get_logger().info(
            f"Received path with {len(msg.poses)} poses"
        )

        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                "NavigateThroughPoses action server not available"
            )
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = msg.poses

        self.goal_sent = True

        self.get_logger().info("Sending path to Nav2 NavigateThroughPoses")

        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            self.goal_sent = False
            return
        
        self.get_logger().info("Nav2 goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):

        result = future.result().result
        status = future.result().status

        self.get_logger().info(
            f"Nav2 finished with status: {status}"
        )
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status('goal succeeded')
            reached = Bool()
            reached.data = True
            self.reached_pub.publish(reached)
            return
        else:
            self.publish_status('goal failed')

        self.goal_sent = False

    def publish_status(self, text):
        if not self.active and text != 'stopped':
            return

        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

def main(args=None):

    rclpy.init(args=args)

    node = NavigatorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()



