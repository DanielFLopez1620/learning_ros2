from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle_node'
        ),
        Node(
            package='m03_tf2_with_cpp',
            executable='turtle_broadcaster',
            name='orginial_broadcaster',
            parameters=[
                {'turtlename': 'follower'}
            ]
        )
    ])