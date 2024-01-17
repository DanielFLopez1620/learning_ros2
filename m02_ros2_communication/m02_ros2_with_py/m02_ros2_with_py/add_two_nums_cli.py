# !/usr/bin/env python3

# Sys is needed to read args passed with the executable
import sys

# ------------------ ROS2 Dependencies ----------------------------------------
import rclpy
from rclpy.node import Node

# ------------------ ROS2 Message/Srv Definition -----------------------------
from example_interfaces.srv import AddTwoInts

# ------------------ Client Node Class ----------------------------------------
class SimpleAdderCli(Node):
    """
    Simple class related with addition clients

    Attributes
    ---
    cli : Client object
        Client that will make request (two nums) using the '/add_two_ints'
    req : Request object
        Request that contains two double nums

    Methods
    ---
    cli_request(a,b)
        Make request to obtain sum of 'a' and 'b'
    """
    def __init__(self):
        """
        Constructor that initialize the client node, sets the client for the
        addition of two nums and wait until service is ready.
        """
        super().__init__("add_two_nums_cli")
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Waiting for server...')
        self.req = AddTwoInts.Request()

    def cli_request(self, a, b):
        """
        Sends the client request for the sum of two numbers and return the
        result obtained.

        Params
        ---
        a : float
            First number to sum
        b : float
            Second number to sum

        Returns
        ---
        result : float
            Sum of the two numbers
        """
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_comple(self, self.future)
        return self.future.result()

# ---------------------------- Implementation --------------------------------
def main():
    # Initialize ROS Client Library for Python
    rclpy.init()

    # Intance adder client object
    cli_adder = SimpleAdderCli()
    
    # Get response by waiting to request to be received
    response = cli_adder.send_request(int(sys.argv[1]), int(sys.argv[2]))

    # Log result obtainer
    cli_adder.get_logger().info('Result of the addition is: %d', response.sum)

    # Destroyed of class and shutdown of resources used
    cli_adder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()