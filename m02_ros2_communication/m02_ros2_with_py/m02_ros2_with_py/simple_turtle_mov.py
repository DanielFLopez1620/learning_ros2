# !/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class SimpleTurtleMove(Node):
    """
    Class oriented to simple cmd_vel commands to turltes in turtlesim
    
    Params
    ---
    publisher : rclpy publisher
        Publisher oriented to Twist messages
        
    Methods
    ---
    single_publish():
        Publish a simple cmd_vel command
    """
    def __init__(self):
        """
        Constructor that initialize a node called 'simple_turtle_mov' and create 
        a publisher.
        """
        super().__init__('simple_turtle_mov')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def single_publish(self):
        """
        Single publish a command velocity to the turtle
        """
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 3.1416 / 2
        self.publisher_.publish(msg)



def main(args=None):
    """
    Simple program that serves as a example of a single minimal publisher that sends
    a velocity command to the turtlesim.

    Make sure the turtlesim is available before running this node.

        ros2 run turtlesim turtlesim_node

    You can run this node, after the colcon construction and running the source, with:

        ros2 run m03_ros2_with_py
    """
    # Initialize ROS Client Library for Python
    rclpy.init(args=args)
    
    # Instance object and launch command
    my_command = SimpleTurtleMove()
    my_command.single_publish()

    # Spin to make sure the publish is completed
    rclpy.spin(my_command)

    # Close and destroy node
    my_command.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
