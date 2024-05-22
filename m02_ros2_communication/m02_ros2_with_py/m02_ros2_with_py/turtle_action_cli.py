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
        
        self._send_goal_future = self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal wasn\'t accepted')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Num of moves achieved: {0}'.format(result.moves))
        rclpy.shutdown()
    

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Current move: {0}'.format(feedback.current_move))


def main(args=None):
    rclpy.init(args=args)
    turtle_action_client = TurtleActionClient()
    turtle_action_client.send_goal(5)
    rclpy.spin(turtle_action_client)

if __name__ == '__main__':
    main()
