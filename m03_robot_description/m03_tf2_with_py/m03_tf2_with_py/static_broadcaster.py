# !/usr/bin/env python3

# ------------------------- Python Standard Libraries -------------------------
import math
import sys
import numpy as np
import random
import time

# ------------------------ ROS2 rclpy implementation --------------------------
import rclpy
from rclpy.node import Node

# ------------------------ ROS2 Messages and Tf2 related ----------------------
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
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

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change, because it is static
    
    Methods
    ---
    make_transforms(transformation)
        Generate static transform considering child, xyz and rpy.
    """
    def __init__(self, transformation):
        """
        Constructor that initialize a node with the specified game and starts
        a static transform.
        
        Params
        ---
        transformation : Array of size 7 (child_frame_name x y z roll pitch yaw).
            Array with the data to consider in the creation of the static
            transform.
        """
        # Initialize node
        super().__init__('static_turtle_tf2_broadcaster')

        # Instance a static transform
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Generate the transform with the given arguments in tf array passed
        self.make_transforms(transformation)
        

    def make_transforms(self, transformation):
        """
        Generate transformation by adding the info related with 
        child_frame_name, x, y, z, roll, pitch, yaw and it also adds a time
        stamp to generate the TransformStamped type.
        
        Params
        ---
        transformation : Array of size 7 (child_frame_name x y z roll pitch yaw).
            Array with the data to consider in the creation of the static
            transform.
        """
        # Instance transform
        t = TransformStamped()

        # Consider time stamp
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = transformation[1]

        # Add the proper information of the transform (it includes a change
        # from rpy to quaternion)
        t.transform.translation.x = float(transformation[2])
        t.transform.translation.y = float(transformation[3])
        t.transform.translation.z = float(transformation[4])
        quat = quaternion_from_euler(
            float(transformation[5]), float(transformation[6]), float(transformation[7]))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Finally, send transform
        self.tf_static_broadcaster.sendTransform(t)


def main():
    logger = rclpy.logging.get_logger('logger')

    # obtain parameters from command line arguments
    if len(sys.argv) != 2:
        logger.info('Invalid number of parameters. Usage: \n'
                    '$ ros2 run learning_tf2_py static_turtle_tf2_broadcaster'
                    'child_frame_name')
        sys.exit(1)

    if sys.argv[1] == 'world':
        logger.info('Your static turtle name cannot be "world"')
        sys.exit(2)

    transform_data = []
    transform_data.append(sys.argv[1])
    
    # Random number generation 
    random.seed(int(time.time()))
    random_int = random.randrange(0, 10)
    for i in range(3):
        transform_data.append(random_int)
    random_float = random.uniform(0, math.pi)
    for j in range(3):
        transform_data.append(random_float)
    
    # Initialize ROS Client Library for Python
    rclpy.init()
    
    # Instance node and then spin until Keyboard Interrupt
    node = StaticFramePublisher(sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # 
    rclpy.shutdown()