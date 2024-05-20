import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from m02_ros2_with_cpp.action import RegularMove

class TurtleActionClient(Node):
    def __init__(self):
        super().__init__('turtle_action_cli')
        self._action_client = ActionClient(self, RegularMove, 'turtle_mov')

    def send_goal(self, moves):
        goal = RegularMove.Goal()
        goal.num_moves = moves

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal)
    
def main(args=None):
    rclpy.init(args=args)

    turtle_action_client = TurtleActionClient()

    wait_for_result = turtle_action_client.send_goal(10)

    rclpy.spin_until_future_complete(turtle_action_client, wait_for_result)
