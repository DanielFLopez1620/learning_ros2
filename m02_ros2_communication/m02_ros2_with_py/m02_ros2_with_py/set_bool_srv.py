# !/usr/bin/env python3

# ---------------------------- ROS2 Dependencies ------------------------------
import rclpy
from rclpy.node import Node

# ---------------------------- ROS2 Msg/Srv -----------------------------------
from std_srvs.srv import SetBool

# --------------------------- Python Dependencies -----------------------------
from random import seed, randint
import time

# ---------------------------- Server Class Node ------------------------------
class BoolSetterSrv(Node):
    """
    Simple class that sets a bool

    Attributes
    ---
    srv : Service Object
        Server that attends request made in /just_a_boolean and implements the
        adder callback

    Methods
    ---
    setter_callback(request, response)
        Function called when a request is received, read it, set the boolean and 
        update if it was done without failures
    """
    def __init__(self):
        """
        Constructor that initialize the node and instance a server
        """
        super().__init__('bool_setter_srv')
        self.srv = self.create_service(SetBool, 'just_a_boolean', 
                                       self.setter_callback)

    def setter_callback(self, request, response):
        """
        Callback to set a boolean value, return true and OK if everythin was
        done without error or false and the message error if something happened

        Params
        ---
        request : Request object
            Contains the info of the two numbers to sums
        response : Response object
            For updating the value of the sum
        """
        seed(time.time())
        
        response.success = False if (randint(2,20)%2) == 0 else True
        response.message = "OK" if response.success else "Random error spawned"

        self.get_logger().info("Request: Set bool to %d" % (1 if response.success 
                                                              else 0))

        return response                                                    

# ---------------------------- Implementation ---------------------------------

def main():
    # Initialize ROS Client Library for Python
    rclpy.init()

    # Instance server for boolean setter
    my_setter_srv = BoolSetterSrv()

    # Generate loop to give time to accept incoming requests
    rclpy.spin(my_setter_srv)

    # Destroy node and shut resources
    my_setter_srv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()