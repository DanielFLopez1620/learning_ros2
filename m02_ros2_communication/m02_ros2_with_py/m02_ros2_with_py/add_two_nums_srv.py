# !/usr/bin/env python3

# ---------------------------- ROS2 Dependencies ------------------------------
import rclpy
from rclpy.node import Node
# ---------------------------- ROS2 Msg/Srv -----------------------------------
from example_interfaces.srv import AddTwoInts

# ---------------------------- Server Class Node ------------------------------
class SimpleAdderSrv(Node):
    """
    Simple class related with a server for addition

    Attributes
    ---
    srv : Service Object
        Server that attends request made in /add_two_ints and implements the
        adder callback

    Methods
    ---
    adder_callback(request, response)
        Function called when a request is received, read it and make the sum
    """
    def __init__(self):
        """
        Constructor that initialize the node and instance a server
        """
        super().__init__('add_two_nums_srv')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', 
                                       self.adder_callback)

    def adder_callback(self, request, response):
        """
        Callback to attend request for sum and respond with the total.

        Params
        ---
        request : Request object
            Contains the info of the two numbers to sums
        response : Response object
            For updating the value of the sum
        """
        response.sum = request.a + request.b 
        self.get_logger().info("Request: (%d , %d)" % (request.a, request.b))

# ---------------------------- Implementation ---------------------------------

def main():
    # Initialize ROS Client Library for Python
    rclpy.init()

    # Instance server for addition
    my_adder = SimpleAdderSrv()

    # Generate loop to give time to accept incoming requests
    rclpy.spin(my_adder)

    # Destroy node and shut resources
    my_adder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()