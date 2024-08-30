# ------------------------ PYTHON DEPENDENCIES --------------------------------
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

# ------------------------ LAUNCH STRUCTURE -----------------------------------
def generate_launch_description():
    """
    Script oriented to luanch the turtlesim node, and create the broadcaster
    and listener for turtle 1 and the "follower" turtle which is going to be 
    spawned during the launch execution.
    """
    # Consider the next descriptions
    return LaunchDescription([

        # Call node for turtlesim and turtle1
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle_node'
        ),

        # Add node to broadcast the tf of /turtle1 according to /world
        Node(
            package='m03_tf2_with_cpp',
            executable='turtle_broadcaster',
            name='original_broadcaster',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),

        # Add argument for target frame
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),

        # Add the broadcaster of the follower turtle
        Node(
            package='m03_tf2_with_cpp',
            executable='turtle_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'follower'}
            ]
        ),

        # Add listener node to make the follower, follow turtle1
        # as the provided target frame
        Node(
            package='m03_tf2_with_cpp',
            executable='turtle_time_travel',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),    
    ])