// ---------------------- STANDARD HEADERS REQUIRED ---------------------------
#include <memory>  // For dynamic memory managemet

// ---------------------- CUSTOM PACKAGE DEPENDENCIES -------------------------
#include "m02_ros2_with_cpp/exam_cli_comp.hpp"  // For ExamClient node
#include "m02_ros2_with_cpp/exam_srv_comp.hpp"  // For ExamServer node
#include "m02_ros2_with_cpp/int64_pub_comp.hpp" // For IntPub node
#include "m02_ros2_with_cpp/int64_sub_comp.hpp" // For IntSub node

// ------------------------- MAIN IMPLEMENTATION ------------------------------
int main(int argc, char* argv[])
{
    // For forcing the flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize ROS2 client for C++ and consider any possible arg provided
    rclcpp::init(argc, argv);

    // Executor responsible of the callback of the nodes, that will run from the
    // main thread
    rclcpp::executors::SingleThreadedExecutor st_exec;

    // Configurable option for the nodes, like intra process communication and
    // others, in this case, the default values are left.
    rclcpp::NodeOptions options;
    
    // Instance the classes via shared pointer, then they are not loaded as
    // components properly said, therefore they won't be listed  with the
    // ros2 component cli commands.
    auto publisherNode = std::make_shared<example_comp::IntPub>(options);
    auto subscriberNode = std::make_shared<example_comp::IntSub>(options);
    auto serverNode = std::make_shared<example_comp::ExamServer>(options);
    auto clientNode = std::make_shared<example_comp::ExamClient>(options);

    // Add nodes to the executor
    st_exec.add_node(publisherNode);
    st_exec.add_node(subscriberNode);
    st_exec.add_node(serverNode);
    st_exec.add_node(clientNode);

    // Spin block of nodes, execute work when it is available, and block again.
    st_exec.spin();

    // Final shutdown and return
    rclcpp::shutdown();
    return 0;

} // main