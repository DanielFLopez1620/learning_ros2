# ------------------------- Launch dependencies -------------------------------
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

# -------------------------- Launch structure ---------------------------------
def generate_launch_description():
    """
    Script oriented to launch load two compontents into an already created
    container, it will load the custom components of ExamClient and ExamSrv
    to the "container_of_comp" container.
    """

    # Continer call
    container = Node(
        name='container_of_comp',
        package='rclcpp_components',
        executable='component_container',
        output='both'
    )
    
    # Load components
    load_composable_nodes = LoadComposableNodes(
        # You can change the target container, but it must exist
        target_container='container_of_comp',
        # Listing components
        composable_node_descriptions=[
            # ExamSrv component (server oriented to check answer or guesses)
            ComposableNode(
                package="m02_ros2_with_cpp",
                plugin='example_comp::ExamServer',
                name="exam_srv",
            ),
            # ExamCli component (client oriented to send answers or guesses)
            ComposableNode(
                package="m02_ros2_with_cpp",
                plugin='example_comp::ExamClient',
                name="exam_cli",
            ),
        ],
    )

    # Add executables and processes
    return LaunchDescription([
        container, 
        load_composable_nodes
    ])