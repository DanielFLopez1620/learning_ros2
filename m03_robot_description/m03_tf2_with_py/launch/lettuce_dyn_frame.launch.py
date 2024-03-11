import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('m03_tf2_with_py'), 'launch'),
            '/tf2_demo.launch.py']),
        launch_arguments={'target_frame': 'lettuce'}.items(),
        )

    return LaunchDescription([
        demo_nodes,
        Node(
            package='m03_tf2_with_py',
            executable='lettuce_stick_frame',
            name='dynamic_broadcaster',
        ),
    ])