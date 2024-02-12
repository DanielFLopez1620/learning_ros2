# !/usr/bin/env python3

# ------------------------------- ROS2 Dependencies ---------------------------
import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor

#-------------------------- Printer Class related with Params -----------------
class SayingHi(rclpy.node.Node):
    """
    Class oriented to print message of hi, as a presentation of itself and by
    adding a name.
    
    Attributes
    ---
    timer - Timer object
        Time counter that links a callback when the period is completed.
        
    Methods
    ---
        saying_callback()
            Print a little presentation with a hi, and the name indicated in
            the params associated.
    """
    
    def __init__(self):
        """
        Constructor that initialize the node (by passing a name), instance a
        param with its description and then declares it to make it accesible
        for other nodes.
        """
        super().__init__("saying_your_name")
        param_desc = ParameterDescriptor(description = "Put here your name")
        self.declare_parameter('your_name', 'ROS User', param_desc)
        self.timer = self.create_timer(1, self.saying_callback)
        
    def saying_callback(self):
        """
        Callback related with the printing of the message, saying hi to the
        user, and updating its value to the original one if a change is 
        received.
        """
        change_param = self.get_parameter('your_name').get_parameter_value().string_value
        self.get_logger().info('Hi! I am node and you are %s' % str(change_param))
        new_param = rclpy.parameter.Parameter('your_name', rclpy.Parameter.Type.STRING, 'ROS User')
        self.set_parameters([new_param])
        
# --------------------------- Implementation ----------------------------------
def main():
    """
    Simple program that iterates a saying hi object actions to wave to the 
    user.
    """
    
    # Initialize ROS LCient Library for Python
    rclpy.init()
    
    # Instance object to print message to the user
    node = SayingHi()
    
    # Spin node
    rclpy.spin(node)
    
    # It is a good practice to destroy the node and shutdown rclpy, but if it
    # forgotten, the garbage collector will take care of it (in the future)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()