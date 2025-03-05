import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    resources_package = 'arrow_bot'
    controllers_names = ["diff_drive_controller", "joint_state_broadcaster"]

    # Make path to resources dir without last package_name fragment.
    path_to_share_dir_clipped = ''.join(get_package_share_directory(resources_package).rsplit('/' + resources_package, 1))
    # Gazebo hint for resources.
    os.environ['GZ_SIM_RESOURCE_PATH'] = path_to_share_dir_clipped
    # Ensure `SDF_PATH` is populated since `sdformat_urdf` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]
        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value='true')

    gazebo_and_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(resources_package),
                'launch',
                'gazebo.launch.py',
            ]),
        ]),
        condition=IfCondition(use_rviz),
        launch_arguments=dict(
            use_sim_time=use_sim_time, 
            use_rviz=use_rviz,
            ).items()
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(resources_package),
            "config",
            "Robot_controllers.yaml",
        ]
    )

    # ROS2 Jazzy gives conflict error if to launch ros2_control_node. It launches it automatically.
    control_node = DeclareLaunchArgument('', default_value='') # dummy for LaunchDescription could take empty element
    if len(controllers_names):
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{
                'robot_controllers': robot_controllers, 
                'use_sim_time': use_sim_time,
            }],
            output="both",
        )

    robot_controller_spawner = DeclareLaunchArgument('', default_value='') # dummy for LaunchDescription could take empty element
    if len(controllers_names):
        robot_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "joint_state_broadcaster"],
            parameters=[{
                'use_sim_time': use_sim_time,
                '--param-file': robot_controllers,
            }],
        )


    rqt = Node(
        package="rqt_gui",
        executable="rqt_gui",
        arguments=['--force-discover'],
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        use_rviz_arg,
        gazebo_and_state,
        # control_node,
        robot_controller_spawner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=robot_controller_spawner,
                on_exit=[
                    rqt
                ]
            )  
        ),
    ])
