# !/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

class MyInt64Subscriber(Node):
    """
    Simple subscriber that search for the /num_int64 info and display it.
    """
    def __init__(self):
        super().__init__("int64_sub")
        self.sub = self.create_subscription(Int64, 'num_int64', self.num_callback, 10)
        self.sub
    """
    Callback implementation when a message is recieved, so you can show it.

    Params
    ---
    msg : std_msgs.msgs/msg/Int64
        The message received, with the 'data' member that storage the integer.
    """
    def num_callback(self, msg):
        self.get_logger().info("Recieving the integer: %d" % msg.data)


"""
Simple program to recieving integers by subscribing to the /num_int64 topic.
"""
def main(args=None):
    rclpy.init(args=args)

    my_sub = MyInt64Subscriber()

    rclpy.spin(my_sub)

    my_sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()