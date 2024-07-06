from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = Node(
        name='container_of_comp',
        package='rclcpp_components',
        executable='component_container',
        output='both'
    )
    
    load_composable_nodes = LoadComposableNodes(
        target_container='container_of_comp',
        composable_node_descriptions=[
            ComposableNode(
                package="m02_ros2_with_cpp",
                plugin='example_comp::ExamServer',
                name="exam_srv",
            ),
            ComposableNode(
                package="m02_ros2_with_cpp",
                plugin='example_comp::ExamClient',
                name="exam_cli",
            ),
        ],
    )

    return LaunchDescription([container, load_composable_nodes])