import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor

class SayingHi(rclpy.node.Node):
    def __init__(self):
        super().__init__("saying_your_name")
        param_desc = ParameterDescriptor(description = "Put here your name")
        self.declare_parameter('your_name', 'ROS User', param_desc)
        self.timer = self.create_timer(1, self.saying_callback)
        
    def saying_callback(self):
        change_param = self.get_parameter('your_name').get_parameter_value().string_value
        self.get_logger().info("Hi! I am node and you are $s" % change_param)
        new_param = rclpy.parameter.Parameter('your_name', rclpy.Parameter.Type.STRING, 'ROS User')
        self.set_parameters([new_param])
        
def main():
    rclpy.init()
    node = SayingHi()
    rclpy.spin(node)
    
    
if __name__ == '__main__':
    main()