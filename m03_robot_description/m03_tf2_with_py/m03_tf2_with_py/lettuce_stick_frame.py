# !/usr/bin/env python3

# -------------------- Import Python Standard Headers -------------------------
import math

# ------------------- Import ROS2 rclpy implementations -----------------------
import rclpy
from rclpy.node import Node

# ----------------------- Import tf2 related packages -------------------------
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ----------------------- Dynamic frame broadcaster class ---------------------
class LettuceStickBroadcaster(Node):
    """
    Class oriented to create a dynamic (moving) frame, that follows the idea of
    a lettuce on a stick (carrot on stick case from minecraft), as it will act 
    as an objective for others transforms to follow.

    Attributes
    ---
    tf_broadcaster : Transform Broadcaster
        Form to communicate the dynamic transform of interest.
        
    timer : Timer
        Timer that will call an update of the tf values.
        
    Methods
    ---
    broadcast_timer_callback:
        Linked to the timer call for dynamic update of the transform.
    """
    def __init__(self):
        """
        'Constructor that initialize a node called 
        'lettuce_stick_tf2_broadcaster', instance a transform broadcaster and
        then instance a timer linked to tf updates.
        """
        # Initialize node
        super().__init__('lettuce_stick_tf2_broadcaster')
        
        # Intance tf2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Instance timer and link callback
        self.timer = self.create_timer(0.25, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        """
        Publish a dynamic trnasform that will consider the time as the
        variation objetive, it will try to mimic the vibration of a lettuce
        when it is hanging on a stick that is attached to the animal.
        """
        # Time considerations
        seconds, _ = self.get_clock().now().seconds_nanoseconds()
        x = seconds * math.pi

        # Instance transform and consider stamped info
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'lettuce'
        
        # Update properly the 2D transform values
        t.transform.translation.x = 10 * math.sin(x)
        t.transform.translation.y = 10 * math.cos(x)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publish (broadcast) transform
        self.tf_broadcaster.sendTransform(t)


def main():
    """
    Program oriented to create a dynamic transform to be followed. In this case
    an analogy of a lettuce attached to a stick.
    """
    
    # Initialize ROS Client Library for Python
    rclpy.init()
    
    # Create a logger
    logger = rclpy.logging.get_logger('logger')
    logger.info("Initializing lettuce and stick dynamic frame....")
    
    # Instance a node
    node = LettuceStickBroadcaster()
    
    # Spin node until keyboard interrupt happens
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Close and exit program
    logger.info("Closing lettuce and stick dynamic frame....")
    rclpy.shutdown()