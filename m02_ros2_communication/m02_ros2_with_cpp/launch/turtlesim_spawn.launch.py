from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (EmitEvent, ExecuteProcess, LogInfo, 
                            RegisterEventHandler)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                    LocalSubstitution)

def generate_launch_description():
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call /spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 5, y: 5, theta: 3.1416}"'
        ]],
        shell=True
    )
    return LaunchDescription([
        turtlesim_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    ExecuteProcess(
                        cmd=[[
                            FindExecutable(name='ros2'),
                            ' param set /turtlesim_node background_r ',
                            '0'
                        ]],
                        shell=True
                    ),
                    ExecuteProcess(
                        cmd=[[
                            FindExecutable(name='ros2'),
                            ' param set /turtlesim_node background_g ',
                            '255'
                        ]],
                        shell=True
                    ),
                    ExecuteProcess(
                        cmd=[[
                            FindExecutable(name='ros2'),
                            ' param set /turtlesim_node background_b ',
                            '10'
                        ]],
                        shell=True
                    ),
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' is closing the process turtlesim.')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed or Ctrl + C or both'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Someone add a Ctrl+C in the terminal?: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])