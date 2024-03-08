# !/usr/bin/env python3

# --------- Import definitions present in other codes of the package ----------
import math
import sys
import numpy as np

# ------------------------ ROS2 rclpy implementations -------------------------
import rclpy
from rclpy.node import Node

# ----------------------- ROS2 Messages and tf2 related -----------------------
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose

def quaternion_from_euler(ai, aj, ak):
    """
    Mathematic function that calculates the equivalent quaternion by
    considering raw, pitch and yaw.
    
    Params
    ---
    ai : float
        Raw angle or the rotation considering unit i vector.
    aj : float
        Pitch angle or the rotation considering unit j vector.
    ak : float
        Yaw angle or the rotation considering unit k vector.
    
    Returns
    ---
    q : float array
        Quaternion as an array of four components (x, y, z, w).
    """
    # Component extraction and consideration
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    
    # Quaternion definition
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

# --------------- Base class of the program for tf2 broadcaster ---------------
class FramePublisher(Node):

    def __init__(self):
        """ 
        Constructor that initialize a node with the name 
        'turtle_tf2_frame_publisher', declares a parameter related with turtle
        names (future relation with turtlesim), instance a broadcaster and 
        subscribes to the turtle pose.
        """
        # Initialize node
        super().__init__('turtle_tf2_frame_publisher')

        # Declare and acquire `turtlename` parameter for using with turtlesim
        self.turtlename = self.declare_parameter(
          'turtlename', 'turtle').get_parameter_value().string_value

        # Instance a broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the turtle pose
        self.subscription = self.create_subscription(
            Pose,                           # Msg type
            f'/{self.turtlename}/pose',     # Topic
            self.handle_turtle_pose,        # Callback
            1)                              # Queue

    def handle_turtle_pose(self, msg):
        # Instance transform
        t = TransformStamped()

        # Time stamp and tf relation 
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        # Getting turtle linear and angular position (2D consideration)
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # As the turtle only rotates relative to z, we obtain the quaternion
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    # Initialize ROS Client Library for Python
    rclpy.init()
    
    # Create a logger for displaying info
    logger = rclpy.logging.get_logger('logger')
    logger.info("Initialization broadcaster of turtles")
    
    # Instance a node 
    node = FramePublisher()
    
    # Spin the node until Keyboard Interrupt is received
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown and close
    logger.info("Closing broadcaster of turtles")
    rclpy.shutdown()