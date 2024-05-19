import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from m02_ros2_with_cpp.action import RegularMove

class turtleActionServer(Node):
    def __init__(self):
        super().__init__('turtle_action_srv')
        self._action_server = ActionServer(
            self,
            RegularMove,
            'turtle_mov',
            self.move_callback
        )
    
    def move_callback(self, goal_handle):
        self.get_logger().info("Executing goal move...")
        feedback = RegularMove.Feedback()
        feedback.current_move = 0

        for i in range(1, goal_handle.request.num_moves):
            feedback.current_move += 1
            self.get_logger().info('Feedback: {0}'.format(feedback.current_move))
            goal_handle.publish_feedback(feedback)
            time.sleep(1)

        goal_handle.succeed()

        result = RegularMove.Result()
        result.moves = feedback.current_move
        return result

def main(args=None):
    rclpy.init(args=args)
    turtle_action_server = turtleActionServer()
    rclpy.spin(turtle_action_server)

if __name__ == '__main__':
    main()


