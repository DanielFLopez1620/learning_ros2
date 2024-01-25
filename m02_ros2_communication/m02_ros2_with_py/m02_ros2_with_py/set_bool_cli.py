# !/usr/bin/env python3

# Sys is needed to read args passed with the executable
import sys

# ------------------ ROS2 Dependencies ----------------------------------------
import rclpy
from rclpy.node import Node

# ------------------ ROS2 Message/Srv Definition -----------------------------
from std_srvs.srv import SetBool

# ------------------ Client Node Class ----------------------------------------
class BoolSetterCli(Node):
    """
    Simple class related with a boolean setter client

    Attributes
    ---
    cli : Client object
        Client that will make request to set a bool using the '/just_a_boolean'
    req : Request object
        Request that contains the boolean confirmation and message

    Methods
    ---
    cli_request(data)
        Make request to set the 'data' value and receive the respective
        confirmation.
    """
    def __init__(self):
        """
        Constructor that initialize the client node, sets the client for the
        addition of two nums and wait until service is ready.
        """
        super().__init__("bool_setter_cli")
        self.cli = self.create_client(SetBool, 'just_a_boolean')
        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Waiting for server...')
        self.req = SetBool.Request()

    def cli_request(self, data):
        """
        Sends the client request for setting a boolean data (enable/disable),
        and then get a confirmation and logging.

        Params
        ---
        data : bool
            Value to set for enable/disable

        Returns
        ---
        succes : bool
            Indicates if the value was set succesfully
        message : string
            Logging message related with the status of the set
        """
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

# ---------------------------- Implementation --------------------------------
def main():
    # Initialize ROS Client Library for Python
    rclpy.init()

    # Intance adder client object
    cli_setter = BoolSetterCli()
    
    # Get response by waiting to request to be received
    response = cli_setter.cli_request(True if int(sys.argv[1]) > 0 else False)

    # Log result obtained
    cli_setter.get_logger().info('Process was a %s' %
                                'success' if response.success else 'failure')
    cli_setter.get_logger().info('Message received was: %s' % response.message)

    # Destroyed of class and shutdown of resources used
    cli_setter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()