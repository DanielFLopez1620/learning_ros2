#include <memory>

#include "m02_ros2_with_cpp/exam_cli_comp.hpp"
#include "m02_ros2_with_cpp/exam_srv_comp.hpp"
#include "m02_ros2_with_cpp/int64_pub_comp.hpp"
#include "m02_ros2_with_cpp/int64_sub_comp.hpp"

int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor st_exec;
    rclcpp::NodeOptions options;
    
    auto publisherNode = std::make_shared<example_comp::IntPub>(options);
    auto subscriberNode = std::make_shared<example_comp::IntSub>(options);
    auto serverNode = std::make_shared<example_comp::ExamServer>(options);
    auto clientNode = std::make_shared<example_comp::ExamClient>(options);

    st_exec.add_node(publisherNode);
    st_exec.add_node(subscriberNode);
    st_exec.add_node(serverNode);
    st_exec.add_node(clientNode);

    st_exec.spin();

    rclcpp::shutdown();
    return 0;
}