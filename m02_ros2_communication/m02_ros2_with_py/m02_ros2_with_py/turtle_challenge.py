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

# -------------- Class implementation for using turtlesim --------------------
class PlaygroundTurtle(Node):
    """
    Class oriented for turtlesim usage, oriented to generating the subscriptions
    and clients needed for using the functionalities of the turtles.
    
    Attributes
    ---
    
    Methods
    ---
    """
    def __init__(self):
        """
        Constructor that initialize the node with name 'playground_turtle' and
        defines the generic client, request, response, publisher and subscriber
        objects for connecting with turtlesim.
        """
        super().__init__('playground_turtle_py')
        self.cli = None
        self.req = None
        self.res = None
        self.pub = None
        self.sub = None
        self.pose = Pose()
    
    
    def clear_board(self):
        """
        Call service that clear the background of the turtlesim.
        
        Returns
        ---
        True if the response is received without errors.
        """
        self.cli = self.create_client(Empty, '/clear')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting turtlesim for action: Clear...")
        self.req = Empty.Request()
        self.res = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.res)
        return True
    
    def spawn_turtle(self, x, y, theta, name):
        """
        Call service that appears new turtles.
        
        Params
        ---
        x : float
            X position in the turtlesim map to spawn.
        y : float
            Y position in the turtlesim map to spawn.
        theta : float
            Orientation of the turtlesim to spawn (radians)
        name : String
            Name of the new turtle.
            
        Returns
        ---
        True if the response is received without errors.
        """
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
        """
        Call service that elimiates a certain turtle.
        
        Params
        ---
        name : String
            Name of the turtle to delete
            
        Returns
        ---
        True if the response is received without errors.
        """
        self.cli = self.create_client(Kill, '/kill')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting turtlesim for action: Kill...")
        self.req = Kill.Request()
        self.req.name = name
        self.res = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.res)
        return True
    
    def teleport_abs_turtle(self, x, y, theta, name):
        """
        Call service that teleport a certain turtle (absolute position to
        turtlesim origin)
        
        Params
        ---
        x : float
            X position in the turtlesim map.
        y : float
            Y position in the turtlesim map.
        theta : float 
            Orientation of the turtle in space (radians).
        name : string
            Name of the turtle to mov
            
        Returns
        ---
            True if the response is received without errors.
        """
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
        """
        Call service that teleport a certain turtle (relative to the
        tranform of the same turtle)
        
        Params
        ---
        v_x : float
            Linear velocity to move oriented to x direction.
        v_theta : float
            Angular velocity to move oriented around z axis.
        name : string
            Name of the turtle to mov.
            
        Returns
        ---
            True if the response is received without errors.
        """
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
        """
        Set color and width of the trace of the given turtle.
        
        Params
        ---
        r : int
            Must be a value between 0 - 255 of red intensity
        g : int
            Must be a value between 0 - 255 of green intensity
        b : int
            Must be a value between 0 - 255 of blue intensity
        width : int
            Specify width of the line trace
        name : string
            Name of the turtle to change trace color
            
        Returns
        ---
            True if the response is received without errors.
        """
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
        """
        Getter of the pose of the turtle by using pose subscription
        """
        self.sub = self.create_subscription(Pose, "/" + name + "/pose", 
            save_pos_turtle, 10)
        
    def save_pos_turtle(self, msg):
        """
        
        """
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