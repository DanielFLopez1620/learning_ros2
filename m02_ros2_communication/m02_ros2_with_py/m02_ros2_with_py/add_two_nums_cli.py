# !/usr/bin/env python3

import sys

from example_interfaces.srv import AddTwoInts
import rclpy
import rclpy.node import Node


class SimpleAdderCli(Node):

    def __init__(self):
        super().__init__("add_two_nums_cli")
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Waiting for server...')
        self.req = AddTwoInts.Request()

    def cli_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_comple(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    intereted_for_adder = SimpleAdderCli()
    
    response = intereted_for_adder.send_request(int(sys.argv[1]), int(sys.argv[2]))

    intereted_for_adder.get_logger().info('Result of the addition is: %d', response.sum)

    intereted_for_adder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()