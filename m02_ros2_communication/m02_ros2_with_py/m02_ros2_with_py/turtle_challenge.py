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
        self.cli = None
        self.req = None
        self.res = None
        self.pub = None
        self.sub = None
        self.pose = Pose()
    
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
    
    def teleport_abs_turtle(self, x, y, theta, name):
        self.cli = self.create_client(TeleportAbsolute, "/" + name + "/teleport_absolute")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting turtlesim for action: Teleport (Abs) " + 
                            str(name) + "...")
        self.req = TeleportAbsolute.Request()
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.res = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.res)
        return True
    
    def teleport_abs_turtle(self, v_x, v_theta, name):
        self.cli = self.create_client(TeleportRelative, "/" + name + "/teleport_relative")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting turtlesim for action: Teleport (Rel) " + 
                            str(name) + "...")
        self.req = TeleportRelative.Request()
        self.req.linear = v_x
        self.req.angulary = v_theta
        self.res = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.res)
        return True
    
    def set_pen_turtle(self, r, g, b, width, name):
        self.cli = self.create_client(SetPen, "/" + name + "/set_pen")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting turtlesim for action: Set Pen " + 
                            str(name) + "...")
        self.req = SetPen.Request()
        self.req.r = r
        self.req.g = g
        self.req.b = b
        self.req.width = width
        self.res = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.res)
        return True
        
    def save_pos_turtle(self, msg):
        self.pose = msg
    
    def get_pose_turtle(self, name):
        self.sub = self.create_subscription(Pose, "/" + name + "/pose", save_pos_turtle)
        
    
    
if __name__ == "__main__":
    pass