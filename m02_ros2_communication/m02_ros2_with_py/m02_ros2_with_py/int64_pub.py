# !/usr/bin/env python3
import random

# ------------------ ROS2 Depedencies ----------------------------------------
import rclpy
from rclpy.node import Node

# -------------------- ROS2 Messages ----------------------------------------
from std_msgs.msg import Int64



class MyInt64Publisher(Node):
    """
    Simple publisher of an integer (64 bit) in a specific time range.
    """
    def __init__(self):
        # Creation of node
        super().__init__('int64_pub')
        
        # Instance a publisher for integer that uses the /num_int64 topic.
        self.pub = self.create_publisher(Int64, 'num_int64', 10)

        # Instance a timer for publishing info at the given time
        time_sleep = 2
        self.timer = self.create_timer(time_sleep, self.num_callback)
    
    def num_callback(self):
        """
        Publish a random integer when the timer indicates it.
        """
        # Instance an integer (64) object.
        msg = Int64()
        msg.data = random.randint(1, 1000)

        # Publish the given message and log the process.
        self.pub.publish(msg)
        self.get_logger().info('Publishing integer: %d' % msg.data )


"""
Simple program to publish an integer with rclpy by implementing a publisher class.
"""
def main(args=None):
    rclpy.init(args=args)
    
    my_pub = MyInt64Publisher()

    rclpy.spin(my_pub)

    my_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()