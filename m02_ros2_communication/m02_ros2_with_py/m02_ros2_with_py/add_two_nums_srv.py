# !/usr/bin/env python3

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

class SimpleAdderSrv(Node):

    def __init__(self):
        super().__init__('add_two_nums_srv')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.adder_callback)

    def adder_callback(self, request, response):
        response.sum = request.a + request.b 
        self.get_logger().info("Request: (%d , %d)" % (request.a, request.b))

def main():
    rclpy.init()

    my_adder = SimpleAdderSrv()

    rclpy.spin(my_adder)

    rclpy.shutdown()

if __name__ == '__main__':
    main()