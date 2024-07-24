# --------------------------- PYthon dependencies -----------------------------
import os
from ament_index_python.packages import get_package_share_directory

# --------------------------- Launch dependenices -----------------------------
from launch import LaunchDescription
from launch_ros.actions import Node

# ---------------------------- Launch structure -------------------------------
def generate_launch_description():
    """
    Script oriented to load the turtlesim node under a namespace with a config
    that is defined in a yaml file.
    """

    # Add .yaml file path for config arguments, the structure must be 
    # compatible with the node of interest
    config = os.path.join(
        get_package_share_directory('m02_ros2_with_cpp'),
        'config',
        'background_colors.yaml'
        )
    
    # List executables and configs
    return LaunchDescription([
        # Add turtlesim node with additional options
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='ros2_with_cpp',
            name='turtlesim',
            parameters=[config]
        )
    ])