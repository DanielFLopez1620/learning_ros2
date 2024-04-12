
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    background_r = LaunchConfiguration("background_r")
    background_g = LaunchConfiguration("background_g")
    background_b = LaunchConfiguration("background_b")
    use_g = LaunchConfiguration("use_g")
    
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
    use_g_launch_arg = DeclareLaunchArgument(
        'use_g',
        default_value='True',
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
    change_background_g = ExecuteProcess(
        cmd=[[
            'ros2 param set turtlesim_node background_g ',
            background_g 
        ]],
        shell=True
    )
    change_background_b = ExecuteProcess(
        cmd=[[
            'ros2 param set turtlesim_node background_b ',
            background_b 
        ]],
        shell=True
    )
    
    change_background_g_after = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                background_b, ' + ', background_r,
                ' >= 400 and ', use_g
            ])
        ),
        cmd=[[
            'ros2 param set turtlesim_node background_g 10' 
        ]],
        shell=True
    )
    
    turtle_challenge = Node(
        executable="turtle_challenge",
        package="m02_ros2_with_py",
        name="turtle_challenge_py",
    )
    
    return LaunchDescription([
        background_r_launch_arg,
        background_g_launch_arg,
        background_b_launch_arg,
        use_g_launch_arg,
        turtlesim_node,
        change_background_r,
        change_background_g,
        change_background_b,
        TimerAction(
            period=2.0,
            actions=[change_background_g_after],
        ),
        TimerAction(
            period=2.5,
            actions=[turtle_challenge]
        )
    ])