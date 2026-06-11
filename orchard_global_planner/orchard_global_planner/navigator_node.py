import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import NavigateThroughPoses


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

        if not self.client.wait_for_server(timerout_sec=15.0):
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
            f"Nav2 finished ith status: {status}"
        )

        self.goal_sent = False
        
def main(args=None):

    rclpy.init(args=args)

    node = NavigatorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()



