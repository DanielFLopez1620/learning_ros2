# !/usr/bin/env python3

# ---------------------------- Python Standard Libreries ----------------------
import time
import random
import math

# ----------------------------- ROS2 Dependencies -----------------------------
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# ----------------------------- Actions and messages ---------------------------
from m02_ros2_with_cpp.action import RegularMove
from geometry_msgs.msg import Twist

# ---------------------- Class implementation for Action Server ----------------
class turtleActionServer(Node):
    """
    Class oriented to create an Action Server to move the turtle1 from turtlesim
    and generate regular polygons patterns according to a client.

    Attributes
    ---
    _action_server : rclpy action server
        Private action server for /turtle_move actions that link the move
        callback.

    Methods
    ---
    move_callback(goal_handle):
        Move the turtle1 when a client request is received, according the 
        action specified.
    """

    def __init__(self): 
        """
        Constructor that initialize the node turtle_action_srv and instance a
        action server for the Regular Move actions under the channel of 
        /turtle mov
        """
        # Initialization using parents constructor
        super().__init__('turtle_action_srv')

        # Action server instance
        self._action_server = ActionServer(
            self,
            RegularMove,
            'turtle_mov',
            self.move_callback
        )
    
    def move_callback(self, goal_handle):
        """
        Callback that will be used to indicate the proper move of the turtle1
        according the specified goal in the action, while also submitting the
        feedback of the operation.

        Params
        ---
        goal_handle : Regular Move Goal Handler
            Contains the structure of an accepted goal by the action server.
        
        Return
        ---
        result : Regular Move Result
            Value obtained after completation of the process.
        """
        self.get_logger().info("Executing goal move...")

        # Instance a feedback and initialize it
        feedback = RegularMove.Feedback()
        feedback.current_move = 0

        # Create a pseudo-random number for the x linear move
        random.seed(time.time())
        dist = random.uniform(0.5, 2)

        # Instance the cmd_vel_publisher to turtle1 (considering Twist msg)
        cmd_vel_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        
        # Instance the Twist msg for movement and define the regular rotation
        msg = Twist()
        rot = 2 * math.pi / goal_handle.request.num_moves

        # Loop linked to the goal to achieve the operation
        for i in range(1, goal_handle.request.num_moves + 1):

            # Rotate the turtle (angle according regular polygon selected)
            msg.linear.x = float(0)
            msg.angular.z = rot
            cmd_vel_publisher.publish(msg)
            time.sleep(1)
            
            # Move linearly according pseudo random generation
            msg.linear.x = dist
            msg.angular.z = float(0)
            cmd_vel_publisher.publish(msg)
            time.sleep(1)

            # Update and publish feedback
            feedback.current_move = i
            self.get_logger().info('Current Move: {0}'.format(feedback.current_move))
            goal_handle.publish_feedback(feedback)
            
        # Notifiy completation of goal
        goal_handle.succeed()

        # Create result and return it
        result = RegularMove.Result()
        result.moves = feedback.current_move
        return result

def main(args=None):
    """
    Implementation of a simple of a server for regular polygon moves with
    turtle1 from turtlesim.
    """

    # Initialize ROS Client Library for Python with the current args.
    rclpy.init(args=args)

    # Instance a turtle action server
    turtle_action_server = turtleActionServer()

    # Spin server
    rclpy.spin(turtle_action_server)

    # When done, destroy node and shutdown 
    turtle_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


