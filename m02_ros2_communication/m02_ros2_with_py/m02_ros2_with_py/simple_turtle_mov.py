# !/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class SimpleTurtleMove(Node):

    def __init__(self):
        super().__init__('simple_turtle_mov')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def single_publish(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 3.1416 / 2
        self.publisher_.publish(msg)



"""
Simple program that serves as a example of a single minimal publisher that sends
a velocity command to the turtlesim.

Make sure the turtlesim is available before running this node.

    ros2 run turtlesim turtlesim_node

You can run this node, after the colcon construction and running the source, with:

    ros2 run m03_ros2_with_py
"""
def main(args=None):
    rclpy.init(args=args)
    
    my_command = SimpleTurtleMove()
    my_command.single_publish()

    rclpy.spin(my_command)

    my_command.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
