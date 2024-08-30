# --------------------------- PYTHON DEPENDENCIES -----------------------------
import os
from ament_index_python.packages import get_package_share_directory

# --------------------------- LAUNCH DEPENDENCIES -----------------------------
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ----------------------------- LAUNCH STRUCTURE -----------------------------
def generate_launch_description():
    """
    Script oriented to call the tf2 rclpy base demo, and add a static frame
    that tends to assimilate the carrot and stick fixed like in minecraft.
    """

    # Include launch for the tf2 base demo
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('m03_tf2_with_py'), 'launch'),
            '/tf2_demo.launch.py']),
        )
    
    # Add include, nodes and processes
    return LaunchDescription([
        # Add included .launch.py file
        demo_nodes,

        # Add node for the static frame
        Node(
            package='m03_tf2_with_py',
            executable='lettuce_frame',
            name='fixed_broadcaster',
        ),
    ])