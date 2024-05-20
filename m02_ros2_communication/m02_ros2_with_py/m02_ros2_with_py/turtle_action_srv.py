import time
import random
import math
from datatime import datatime

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from m02_ros2_with_cpp.action import RegularMove
from geometry_msgs.msg import Twist

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

        random.seed(datatime.now().timestamp())
        dist = random.uniform(0.5, 2);

        cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        msg = Twist()
        rot = 2 * math.pi / goal_handle.request.num_moves



        for i in range(1, goal_handle.request.num_moves):

            msg.linear.x = 0
            msg.angular.z = rot
            cmd_vel_publisher.publish(msg)
            time.sleep(1)

            msg.linear.x = dist
            msg.angular.z = 0
            cmd_vel_publisher.publish(msg)
            time.sleep(1)

            feedback.current_move = i
            self.get_logger().info('Current Move: {0}'.format(feedback.current_move))
            goal_handle.publish_feedback(feedback)
            

        goal_handle.succeed()

        result = RegularMove.Result()
        result.moves = feedback.current_move
        return result

def main(args=None):
    rclpy.init(args=args)
    turtle_action_server = turtleActionServer()
    rclpy.spin(turtle_action_server)

    turtle_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


