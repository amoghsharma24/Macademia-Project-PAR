import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


from math import atan2

from ompl import base as ob
from ompl import geometric as og


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        self.map_data = None

        self.current_x = None
        self.current_y = None

        self.goal_x = None
        self.goal_y = None

        self.plan_requested = False
        self.map_received = False

        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback,
            10
        )

        self.localisation_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.controller_sub = self.create_subscription(
            PoseStamped, 
            '/next_pose',
            self.goal_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/planned_markers',
            10
        )

    def quaternion_to_yaw(self, q):

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        return atan2(
            siny_cosp,
            cosy_cosp
        )
    
    def try_plan(self):

        if self.map_data is None:
            return

        if self.current_x is None:
            return
        
        if self.goal_x is None:
            return

        self.plan_requested = False

        self.plan_demo_path()
    
    def map_callback(self, msg): 
        self.map_data = msg

        self.map_received = True

        self.get_logger().info(
            f"Received map with resolution: {self.map_data.info.resolution}"
        )

        self.try_plan()
        # # only plan once
        # if not hasattr(self, 'planned_once'):

        #     self.planned_once = True

        #     self.try_plan()
    
    def odom_callback(self, msg): 
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)

        self.try_plan()

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        q = msg.pose.pose.orientation
        self.goal_yaw = self.quaternion_to_yaw(q)

        self.plan_requested = True  

        if not hasattr(self, 'map_received'):
            self.try_plan()

    def is_state_valid(self, state):

        if self.map_data is None:
            return False

        x = state.getX()
        y = state.getY()

        info = self.map_data.info

        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height

        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)

        # bounds check
        if map_x < 0 or map_x >= width:
            return False

        if map_y < 0 or map_y >= height:
            return False

        index = map_y * width + map_x

        occupancy = self.map_data.data[index]

        # Occupied or unknown
        if occupancy > 50 or occupancy == -1:
            return False

        return True

    def plan_demo_path(self):

        space = ob.SE2StateSpace()

        info = self.map_data.info

        resolution = info.resolution
        width = info.width
        height = info.height

        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        map_width_m = width * resolution
        map_height_m = height * resolution

        self.get_logger().info(
            f"Map origin: ({origin_x}, {origin_y})"
        )

        self.get_logger().info(
            f"Map size: {map_width_m} x {map_height_m}"
        )

        bounds = ob.RealVectorBounds(2)

        bounds.setLow(0, origin_x)
        bounds.setLow(1, origin_y)

        bounds.setHigh(0, origin_x + map_width_m)
        bounds.setHigh(1, origin_y + map_height_m)

        space.setBounds(bounds)

        ss = og.SimpleSetup(space)

        ss.setStateValidityChecker(self.is_state_valid)

        start = space.allocState()
        start.setX(self.current_x)
        start.setY(self.current_y)
        start.setYaw(self.current_yaw)

        goal = space.allocState()
        goal.setX(self.goal_x)
        goal.setY(self.goal_y)
        goal.setYaw(self.goal_yaw)

        print(
            "start valid:",
            self.is_state_valid(start)
        )

        print(
            "goal valid:",
            self.is_state_valid(goal)
        )

        ss.setStartAndGoalStates(start, goal)

        planner = og.RRTConnect(
            ss.getSpaceInformation()
        )

        ss.setPlanner(planner)

        solved = ss.solve(1.0)

        if solved:

            self.get_logger().info('Path found!')

            path_msg = Path()
            path_msg.header.frame_id = 'map'

            solution = ss.getSolutionPath()
            self.publish_path_markers(solution)

            print(type(solution.getState(0)))

            for i in range(solution.getStateCount()):

                state = solution.getState(i)

                print(type(state))
                print(dir(state))                

                pose = PoseStamped()

                pose.header.frame_id = 'map'

                pose.pose.position.x = state.getX()
                pose.pose.position.y = state.getY()

                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)

        else:
            self.get_logger().warn('No path found')

    def publish_path_markers(self, solution):

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "planned_path"
        marker.id = 0

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for i in range(solution.getStateCount()):

            state = solution.getState(i)

            p = Point()

            p.x = state.getX()
            p.y = state.getY()
            p.z = 0.0

            marker.points.append(p)

        self.marker_pub.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = PlannerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
