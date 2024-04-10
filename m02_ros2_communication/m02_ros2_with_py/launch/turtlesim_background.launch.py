
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    background_r = LaunchConfiguration("background_r")
    background_g = LaunchConfiguration("background_g")
    background_b = LaunchConfiguration("background_b")
    
    background_r_launch_arg = DeclareLaunchArgument(
        'background_r',
        default_value='200',
        description="Red color of the turtlesim background")
    background_g_launch_arg = DeclareLaunchArgument(
        'background_g',
        default_value='200',
        description="Green color of the turtlesim background")
    background_b_launch_arg = DeclareLaunchArgument(
        'background_b',
        default_value='200',
        description="Blue color of the turtlesim background")
    
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node'
    )
    
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set turtlesim_node background_r ',
            background_r 
        ]],
        shell=True
    )
    
    return LaunchDescription([
        background_r_launch_arg,
        background_g_launch_arg,
        background_b_launch_arg,
        turtlesim_node,
        change_background_r
        
    ])