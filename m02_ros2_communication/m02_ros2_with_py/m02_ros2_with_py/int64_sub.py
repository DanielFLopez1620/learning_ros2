# !/usr/bin/env python3

# ---------------------------- ROS2 Dependencies ------------------------------
import rclpy
from rclpy.node import Node

# ---------------------------- ROS2 Messages -----------------------------------
from std_msgs.msg import Int64

# ---------------------------- Subscriber Class Node ---------------------------
class MyInt64Subscriber(Node):
    """
    Simple subscriber that search for the /num_int64 info and display it.

    Attributes
    ---
    sub : Subscriptor object
        Object to subscribe to the /num_int64 topic and read the messages

    Methods
    ---
    num_callback(msg)
        Subscriber callback that display received integer value
    """
    def __init__(self):
        """
        Constructor that initialize node and instance subscriber
        """
        super().__init__("int64_sub")
        self.sub = self.create_subscription(Int64, 'num_int64', 
                                            self.num_callback, 10)
        self.sub
    
    def num_callback(self, msg):
        """
        Callback implementation when a message is recieved, so you can show it.

        Params
        ---
        msg : std_msgs.msgs/msg/Int64
            The message received, with the 'data' member that storage the 
            integer.
        """
        self.get_logger().info("Recieving the integer: %d" % msg.data)


# ------------------------------ Implementation -------------------------------
def main(args=None):
    """
    Simple program to recieving integers by subscribing to the /num_int64 
    topic.
    """

    # Initialize ROS Client Library for Python
    rclpy.init(args=args)

    # Instance subscriber object
    my_sub = MyInt64Subscriber()

    # Spin that acts as a loop
    rclpy.spin(my_sub)

    # Final steps before closing program
    my_sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()