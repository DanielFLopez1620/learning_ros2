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
    cli : rclpy client
        Client implementation
        
    req : request
        Generic request for interaction between service and client
        
    res : response
        Generic request for interaction between service and client
        
    pub : rcply publisher
        Publisher implementation
        
    sub : rcply subscriber
        Subscriber implementation
    
    Methods
    ---
    clear_board():
        Clear screen of turtlesim
    
    spawn_turtle(x, y, theta, name):
        Create new turtle 
    
    kill_turtle(name):
        Delete given turtle
    
    teleport_abs_turtle(x, y, theta, name):
        Move turtle according world origin
    
    teleport_rel_turtle(v_x, v_theta, name):
        Move turtle relative to its tf with velocity
    
    set_pen_turtle(r, g, b, width, name):
        Set color and width trace of the turtle
    
    get_pose_turtle(name):
        Getter of pose of the given turtle
        
    save_pose_turtle(msg):
        Callback to store pose
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
        self.req.x = float(x)
        self.req.y = float(y)
        self.req.theta = float(theta)
        self.req.name = str(name)
        self.res = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.res)
        return True
    
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
        self.req.name = str(name)
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
        self.cli = self.create_client(TeleportAbsolute, "/" + str(name) + 
                                        "/teleport_absolute")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting turtlesim for action: Teleport (Abs) " + 
                            str(name) + "...")
        self.req = TeleportAbsolute.Request()
        self.req.x = float(x)
        self.req.y = float(y)
        self.req.theta = float(theta)
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
        self.cli = self.create_client(TeleportRelative, "/" + str(name) + 
                                        "/teleport_relative")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting turtlesim for action: Teleport (Rel) " + 
                            str(name) + "...")
        self.req = TeleportRelative.Request()
        self.req.linear = float(v_x)
        self.req.angulary = float(v_theta)
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
        self.req.r = int(r)
        self.req.g = int(g)
        self.req.b = int(b)
        self.req.width = int(width)
        self.res = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.res)
        return True
        
    def get_pose_turtle(self, name):
        """
        Getter of the pose of the turtle by using pose subscription via 
        callback
        
        Params
        ---
        name : string
            Name of the turtle to obtain the pose
        """
        self.sub = self.create_subscription(Pose, "/" + str(name) + "/pose", 
            save_pos_turtle, 10)
        
    def save_pos_turtle(self, msg):
        """
        Callback for getting the pose of the given turtle.
        
        Params
        ---
        msg : Turtle Pose
            Pose obtained via message of the turtle
        """
        self.pose = msg
        
    
def main():
    """
    Main oriented to test the playground functions that interacts with 
    turtlesim.
    """
    # Initialize ROS Client Library for Python
    rclpy.init()
    
    # Define name and instance class
    name1 = "dan_turtle"
    my_turtle = PlaygroundTurtle()
    
    # Start playing with turtlesim
    my_turtle.clear_board()
    my_turtle.spawn_turtle(3, 3, 0, name1)
    my_turtle.teleport_abs_turtle(7, 3, 3*math.pi/2, name1)
    my_turtle.set_pen_turtle(255, 0, 0, 2, name1)
    my_turtle.teleport_abs_turtle(7, 7, -math.pi, name1)
    my_turtle.set_pen_turtle(0, 255, 0, 1, name1)
    my_turtle.teleport_abs_turtle(3, 7, math.pi/2, name1)
    my_turtle.set_pen_turtle(0, 0, 255, 2, name1)
    my_turtle.teleport_abs_turtle(3, 3, math.pi/2, name1)
    my_turtle.kill_turtle(name1)
    
    # Destroy node and close program
    my_turtle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()