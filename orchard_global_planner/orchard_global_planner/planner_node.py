import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

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

        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback,
            10
        )
    
    def map_callback(self, msg): 
        self.map_data = msg

        self.get_logger().info(
            f"Received map with resolution: {self.map_data.info.resolution}"
        )

        # only plan once
        if not hasattr(self, 'planned_once'):

            self.planned_once = True

            self.plan_demo_path()

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

        bounds = ob.RealVectorBounds(2)

        bounds.setLow(0, origin_x)
        bounds.setLow(1, origin_y)

        bounds.setHigh(0, origin_x + map_width_m)
        bounds.setHigh(1, origin_y + map_height_m)

        space.setBounds(bounds)

        ss = og.SimpleSetup(space)

        ss.setStateValidityChecker(self.is_state_valid)

        start = space.allocState()
        start.setX(1.0)
        start.setY(1.0)
        start.setYaw(0.0)

        goal = space.allocState()
        goal.setX(8.0)
        goal.setY(8.0)
        goal.setYaw(0.0)

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


def main(args=None):

    rclpy.init(args=args)

    node = PlannerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
