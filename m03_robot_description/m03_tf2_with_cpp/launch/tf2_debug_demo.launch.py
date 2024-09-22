from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
         'target_frame', default_value='turtle1',
         description='Target frame name.'
      ),
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         name='sim',
         output='screen'
      ),
      Node(
         package='m03_tf2_with_cpp',
         executable='turtle_broadcaster',
         name='turtle1_broadcaster',
         parameters=[
               {'turtlename': 'turtle1'}
         ]
      ),
      Node(
         package='m03_tf2_with_cpp',
         executable='turtle_broadcaster',
         name='follower_broadcast',
         parameters=[
               {'turtlename': 'follower'}
         ]
      ),
      Node(
         package='m03_tf2_with_cpp',
         executable='turtle_listen_debug',
         name='listener_debug',
         parameters=[
               {'target_frame': LaunchConfiguration('target_frame')}
         ]
      ),
   ])