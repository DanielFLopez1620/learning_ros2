# ----------------------- PYTHON DEPENDENCIES ---------------------------------
import os
from ament_index_python.packages import get_package_share_directory

# ----------------------- LAUNCH DEPENDENCIES ---------------------------------
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ---------------------- LAUNCH STRUCTURE -------------------------------------
def generate_launch_description():
    """
    Script oriented to call the tf2 rclpy base demo, and add a dynamic frame
    that tends to assimilate a moving lettuce with a lot of wind put on a stick
    to consider the turtle moving to it.
    """

    # Include launch of tf demo
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('m03_tf2_with_py'), 'launch'),
            '/tf2_demo.launch.py']),
        launch_arguments={'target_frame': 'lettuce'}.items(),
        )

    # Add luanches, executables and processes
    return LaunchDescription([
        # Add includend .launch.py file
        demo_nodes,

        # Add node for the dynamic broadcaster
        Node(
            package='m03_tf2_with_py',
            executable='lettuce_stick_frame',
            name='dynamic_broadcaster',
        ),
    ])