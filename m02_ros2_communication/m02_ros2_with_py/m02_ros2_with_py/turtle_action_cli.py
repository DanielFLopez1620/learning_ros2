# ----------------------- Python Standard Libraries ---------------------------
import sys

# ----------------------- ROS2 Dependencies -----------------------------------
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# ---------------------- Actions requiered ------------------------------------
from m02_ros2_with_cpp.action import RegularMove

# ----------------- Class Implementation for Turtle Action Client -------------
class TurtleActionClient(Node):
    """
    Class oriented to create an Action Client to move the turtle1 from turtlesim
    and generate regular polygons patterns according to a client.

    Attributes
    ---
    _action_client : rclpy action client
        Private acction client for /turtle_mov actions
    
    _send_goal_future : rclpy task future
        A future instance to a goal handle that completes when the goal is
        accepted or rejected.

    _get_result_future :  rclpy task future
        A future instance that completes when the result is ready
    """
    def __init__(self):
        """
        Simple constructor that initialize the 'turtle_action_cli' node and
        instance an action client for 'turtle_move'.
        """
        # Initialize node from parent 
        super().__init__('turtle_action_cli')

        # Instance action client for regular move via turtle_mov.
        self._action_client = ActionClient(self, RegularMove, 'turtle_mov')

    def send_goal(self, moves):
        """
        Function to send the goal to a server, and wait to know if it is
        received.

        Params
        ---
        moves : int
            Number of vertex to consider to the move and form a regular 
            polygon.
        """
        # Assign goal and update moves to achieve
        goal = RegularMove.Goal()
        goal.num_moves = moves

        # Wait for action server to be ready
        self._action_client.wait_for_server()
        
        # Send asynchronous goal, then you will have a goal_handler
        self._send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        
        # Link callback to indicate goal response status after sending
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """
        Determinates if the goal was accepted or not. If it was accepted, ask
        for the result.

        Param
        ---
        future : rclpy.task.Future
            A future instance of a goal_handle.
        """
        # Obtain goal handle to check if task was accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal wasn\'t accepted')
            return

        # Display accepted, get result and link done callback for getting the
        # result.
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Obtain the result (if nothing unexpected happens), display the resul
        and close node.

        Param
        ---
        future : rclpy.task.Future
            A future instance of a goal_handle.
        """
        # Assign result
        result = future.result().result
        
        # Display moves achieved
        self.get_logger().info('Num of moves achieved: {0}'.format(result.moves))

        # Close everything
        rclpy.shutdown()
    

    def feedback_callback(self, feedback_msg):
        """
        Publish feedback when called during the execution of the goal.

        Param
        ---
        feedback_msg : Regular Move Feedback
            Contains the current move of the Regular Move action in process.
        """
        # Assign feedback only 
        feedback = feedback_msg.feedback

        # Display current move
        self.get_logger().info('Current move: {0}'.format(feedback.current_move))


def main():
    # Initialize ROS2 Client Library for Python 
    rclpy.init()

    # Instance Turtle Action Client Object 
    turtle_action_client = TurtleActionClient()

    # Send goal received
    turtle_action_client.send_goal(int(sys.argv[1]))

    # Spin until completation
    rclpy.spin(turtle_action_client)

if __name__ == '__main__':
    main()
