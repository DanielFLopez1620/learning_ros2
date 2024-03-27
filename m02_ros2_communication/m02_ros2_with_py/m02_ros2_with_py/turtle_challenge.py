# !/usr/bin/env python3

# ------------------ Python Standard Libraries-------------------------------
import random

# ------------------ ROS2 Depedencies ----------------------------------------
import rclpy
from rclpy.node import Node

# -------------------- Turtlesim msg and srvs --------------------------------
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen, Kill
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from std_srvs.srv import Empty

class PlayfulTurtle(Node):
    def __init__(self):
        super().__init__('playful_turtle_py')
        self.pub = None
        self.cli = None
        self.msg = None
        self.req = None
        self.res = None
    
    def clear_board(self):
        self.cli = self.create_client(Empty, '/clear')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting turtlesim for action: Clear...")
        self.req = Empty.Request()
        self.res = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.res)
        return True
    
    def spawn_turtle(self, x, y, theta, name):
        self.cli = self.create_client(Spawn, '/spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting turtlesim for action: Spawn...")
        self.req = Spawn.Request()
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.req.name = name
        self.res = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.res)
        return self.res.name
    
    def kill_turtle(self, name):
        self.cli = self.create_client(Kill, '/kill')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting turtlesim for action: Kill...")
        self.req = Kill.Request()
        self.req.name = name
        self.res = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.res)
        return True
    
    
if __name__ == "__main__":
    pass