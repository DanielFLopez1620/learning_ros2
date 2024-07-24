# -------------------------- Launch dependencies ------------------------------
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# -------------------------- Launch structure ---------------------------------
def generate_launch_description():
    """
    Script oriented to create a component container and add two components that
    are IntPub and SubPub as a showcase of component demostration.
    """
    container = ComposableNodeContainer(
        # Params of the component node
        name='container_of_comp',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        # Adding component nodes
        composable_node_descriptions=[
            # IntPub component, publisher of integer of 64 btis
            ComposableNode(
                package="m02_ros2_with_cpp",
                plugin="example_comp::IntPub",
                name="int64_pub"
            ),
            # IntSub compontent, subscriber of integers of 64 bits
            ComposableNode(
                package="m02_ros2_with_cpp",
                plugin="example_comp::IntSub",
                name="int64_sub"
            ),
        ],
        output='screen',
    )

    # Add executable and process
    return launch.LaunchDescription([container])