import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='container_of_comp',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package="m02_ros2_with_cpp",
                plugin="example_comp::IntPub",
                name="int64_pub"
            ),
            ComposableNode(
                package="m02_ros2_with_cpp",
                plugin="example_comp::IntSub",
                name="int64_sub"
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])