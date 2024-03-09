# !/usr/bin/env python3

# ----------------------- Import of standard libraries ------------------------
import math
import random
import time

# -------------------- Import ROS2 rclpy immplementations ---------------------
import rclpy
from rclpy.node import Node

# -------------------- Import tf2 related packages info -----------------------
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# ---------------- Import of ROS2 msg and srv from other packages -------------
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn


# ----------------- Listener class of transforms (TF2)-------------------------
class FrameListener(Node):
    """
    Class oriented to listening to a given transform and the movements that are
    applied to it.

    Attributes
    ---
    target_frame : String
        Indicates the frame to listen (frame with the tf of interest)
        
    tf_buffer : Transform Buffer
        Buffer that will storage the transform
        
    tf_listener : Transform Listener
        Listener (or subscriber) that will receive the tf info of interest.
        
    spawner : ROS2 Client
        Client that will call the /spawn service to create a new turtle
    
    publisher : ROS2 Publisher
        Publisher that will send data through /cmd_vel of the target turtle.
        
    timer : ROS2 timer
        Timer that will link a callback to listen to the transform
    
    Methods
    ---
    on_timer()
        Callback for the timer
    """
    def __init__(self):
        """
        Constructor that initialize the node with the name 
        'turtle_tf2_frame_listener', create a transform listener and
        instance a cmd_vel publisher for turtles.
        """
        # Initialize node
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'turtle1').get_parameter_value().string_value

        # Create a buffer and a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle and the flags to check the spawn.
        self.spawner = self.create_client(Spawn, 'spawn')
        self.turtle_spawning_service_ready = False
        self.turtle_spawned = False

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names, then they will be computed
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                # Look up for the transform between the two frames and get the info
                try:
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time())
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return

                # Create Twist message with the stimation to reach the other turtle
                msg = Twist()
                scale_rotation_rate = 1.0
                msg.angular.z = scale_rotation_rate * math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x)
                scale_forward_speed = 0.5
                msg.linear.x = scale_forward_speed * math.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2)
                
                # Publish velocity command and start following turtle
                self.publisher.publish(msg)
                self.get_logger().info("Following turtle, you can check tfs")
                
            else:
                # Additional check on spawn call of the turtle
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn was not finished')
        else:
            # In case the turtle doesn't exist, it spawns one in a random pos
            if self.spawner.service_is_ready():
                # Set up random position and fill out the request
                random.seed(int(time.time()))
                request = Spawn.Request()
                request.name = 'turtle2'
                request.x = float(random.randrange(0, 10))
                request.y = float(random.randrange(0, 10))
                request.theta = float(random.uniform(0, math.pi))
                
                # Made the request and update flag
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
                self.get_logger().info("Spawned new listener turtle")
            else:
                # Check if the service is ready
                self.get_logger().info('Turtlesim srvs seems to be offline')


def main():
    # Initialize ROS Client Library for Python
    rclpy.init()
    
    # Create a logger for displaying info
    logger = rclpy.logging.get_logger('logger')
    logger.info("Initialized: Listener of turtles tfs.")
    
    # Instance node
    node = FrameListener()
    
    # Spin node until a keyboard interrupt is detected
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Close program 
    logger.info("Closing: Listener of turtles tfs.")
    rclpy.shutdown()