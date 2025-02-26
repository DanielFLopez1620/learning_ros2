import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    package_name = 'm03_tf2_with_cpp'
    pkg_share = FindPackageShare(
		package=package_name).find(package_name)
    
    print(pkg_share)
    
    rpy_guide = Node(
        package=package_name,
        executable='rpy_guide_for_viz',
        name='rpy_guide_for_viz'
    )
    rpy_logger = Node(
        package=package_name,
        executable='rpy_param_checker',
        name='rpy_param_checker',
        output='screen'
    )

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rpy_guide_vis.rviz')
    print(default_rviz_config_path)
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
		name='rviz_config_file',
		default_value=default_rviz_config_path,
		description='Full path to the RVIZ config file to use')

    start_rviz_cmd = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', rviz_config_file])
    
    ld = LaunchDescription()

    ld.add_action(rpy_guide)
    ld.add_action(rpy_logger)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
