from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        Node(
            package="m02_ros2_with_py",
            exec_name="turtle_simple_mov",
            name='simple_move'
        )
    ])