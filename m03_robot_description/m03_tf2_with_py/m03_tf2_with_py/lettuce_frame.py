# !/usr/bin/env python3

# ------------------- Import ROS2 rclpy implementations -----------------------
import rclpy
from rclpy.node import Node

# ------------------- Import tf2 related packages -----------------------------
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ----------------- Fixed frame class for lettce ------------------------------
class LettuceFrameBroadcaster(Node):
    """
    Class oriented to create a fixed frame that will be used to call the
    attention of the turtles. (Example of adding a frame to the turtle).
    
    Attributes
    ---
    tf_broadcaster : Transform Broadcaster
        Will be used to broadcast the trasnform of the lettuce.
        
    timer : Timer
        Timer that will be linked to an update of the broadcaster.
        
    Methods
    ---
    broadcast_timer_callback()
        Will update the transform of the lettuce according to the turtle.
    """
    def __init__(self):
        """
        Constructor that initialize a node with the name 
        'fixed_lettuce_tf2_broadcaster', instnace a broadcaster and a timer,
        this last one linked to an update function of the broadcast.
        """
        
        # Initialize node 
        super().__init__('fixed_lettuce_tf2_broadcaster')
        
        # Instance broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer and link update callback
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        """
        Broadcast the transform between turtle1 and the lettuce. In this case,
        it will be fixed.
        """
        
        # Instance transform
        t = TransformStamped()

        # Add stamp and relation
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'lettuce'
        
        # Update components of the transform
        t.transform.translation.x = 0.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast (send) transform
        self.tf_broadcaster.sendTransform(t)


def main():
    # Initialize ROS Client Library for Python
    rclpy.init()
    
    # Creating a logger
    logger = rclpy.logging.get_logger('logger')
    logger.info("Initialize lettuce fixed frame...")
    
    # Instance a node
    node = LettuceFrameBroadcaster()
    
    # Spin node until keyboard interrupt is achieved
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Closing program
    logger.info("CLoing lettuce fixed frame...")
    rclpy.shutdown()