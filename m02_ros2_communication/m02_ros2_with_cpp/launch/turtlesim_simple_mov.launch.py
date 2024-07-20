# --------------------------- Launch dependencies -----------------------------
from launch import LaunchDescription
from launch_ros.actions import Node

# -------------------------- Launch structure ---------------------------------
def generate_launch_description():
    """
    Simple script to launch (execute) two nodes in a sequencial and simple way,
    by showcasing the turtles with turtlesim interaction.
    """
    return LaunchDescription([
        # Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        # Simple move node, oriented to a brief implementation with /cmd_vel
        Node(
            package="m02_ros2_with_cpp",
            executable="simple_turtle_mov",
            name='simple_move'
        )
    ])