# !/usr/bin/env python3
import random

# ------------------ ROS2 Depedencies ----------------------------------------
import rclpy
from rclpy.node import Node

# -------------------- ROS2 Messages -----------------------------------------
from std_msgs.msg import Int64

# ------------------- Publisher Class Node ------------------------------
class MyInt64Publisher(Node):
    """
    Simple publisher of an integer (64 bit) in a specific time range.

    Attributes
    ---
    pub : Publisher object
        Object that will communicate Int64 nums by using the /num_int64 topic

    timer - Timer object
        Time counter that links a callback when the period of time is copleted

    Methods
    ---
        num_callback()
            Publish a random number when called
    """
    def __init__(self):
        """
        Constructor that initialize node, set publisher and timer.
        """
        # Initialization of node (name must be unique)
        super().__init__('int64_pub')
        
        # Instance a publisher for integer that uses the /num_int64 topic and 
        # queue size of 10.
        self.pub = self.create_publisher(Int64, 'num_int64', 10)

        # Instance a timer for publishing info at the given time
        time_sleep = 2
        self.timer = self.create_timer(time_sleep, self.num_callback)
    
    def num_callback(self):
        """
        Publish a random integer when the timer indicates it.
        """
        # Instance an integer (64 bits) object and generate a random number
        msg = Int64()
        msg.data = random.randint(1, 1000)

        # Publish the given message and log the process.
        self.pub.publish(msg)
        self.get_logger().info('Publishing integer: %d' % msg.data )


# -------------------------- Implementation -----------------------------------
def main(args=None):
    """
    Simple program to publish an integer with rclpy by implementing a publisher 
    class.
    """

    # Initialize ROS Client Library for Python
    rclpy.init(args=args)
    
    # Instance integer publisher
    my_pub = MyInt64Publisher()

    # Spin (act as a loop everytime a publishing is complete)
    rclpy.spin(my_pub)

    # It is a good practice to destroy the node and shutdown rclpy, but if it
    # forgotten, the garbage collector will take care of it (in the future)
    my_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()