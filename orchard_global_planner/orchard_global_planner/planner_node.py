import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
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

        self.plan_demo_path()

    def is_state_valid(self, state):
        return True

    def plan_demo_path(self):

        space = ob.SE2StateSpace()

        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0)
        bounds.setHigh(10)

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
