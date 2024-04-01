# !/usr/bin/env python3

# ------------------ Python Standard Libraries-------------------------------
import math

# ------------------ ROS2 Depedencies ----------------------------------------
import rclpy
from rclpy.node import Node

# -------------------- Turtlesim msg and srvs --------------------------------
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen, Kill
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from std_srvs.srv import Empty

class PlaygroundTurtle(Node):
    def __init__(self):
        super().__init__('playground_turtle_py')
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
    
    def teleport_rel_turtle(self, v_x, v_theta, name):
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
        
    def get_pose_turtle(self, name):
        self.sub = self.create_subscription(Pose, "/" + name + "/pose", save_pos_turtle, 10)
        
    def save_pos_turtle(self, msg):
        self.pose = msg
    
def main():
    rclpy.init()
    
    name1 = "dan_turtle"
    my_turtle = PlaygroundTurtle()
    my_turtle.clear_board()
    my_turtle.spawn_turtle(3, 3, 0, name1)
    my_turtle.teleport_abs_turtle(7, 3, 3*math.pi()/2, name1)
    my_turtle.set_pen_turtle(255, 0, 0, 2, name1)
    my_turtle.teleport_abs_turtle(7, 7, -math.pi(), name1)
    my_turtle.set_pen_turtle(0, 255, 0, 1, name1)
    my_turtle.teleport_abs_turtle(3, 7, math.pi()/2, name1)
    my_turtle.set_pen_turtle(0, 0, 255, 2, name1)
    my_turtle.teleport_abs_turtle(3, 3, math.pi()/2, name1)
    
    my_turtle.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()