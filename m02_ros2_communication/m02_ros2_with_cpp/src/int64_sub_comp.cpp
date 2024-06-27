#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int64.hpp"
#include "m02_ros2_with_cpp/int64_sub_comp.hpp"

namespace example_comp
{
    IntSub::IntSub(const rclcpp::NodeOptions & options)
    : Node("int64_sub", options)
    {
        auto callback =
            [this](std_msgs::msg::Int64::ConstSharedPtr msg) -> void
            {
                RCLCPP_INFO(this->get_logger(), "Reading num: %ld", msg->data);
                std::flush(std::cout);
            };
        
        sub_ = create_subscription<std_msgs::msg::Int64>("num_int64", 10, callback);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(example_comp::IntSub)
