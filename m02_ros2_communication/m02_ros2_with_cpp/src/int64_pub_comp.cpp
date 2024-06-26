#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "m02_ros2_with_cpp/int64_pub_comp.hpp"

using namespace std::chrono_literals;

namespace example_comp
{
    IntPub::IntPub(const rclcpp::NodeOptions & options)
    : Node("int64_pub_comp", options), element_(0)
    {
        pub_ = create_publisher<std_msgs::msg::Int64>("num_int64",10);
        timer_ = create_wall_timer(500ms, std::bind(&IntPub::on_timer, this));
    }

    void IntPub::on_timer()
    {
        auto msg = std::make_unique<std_msgs::msg::Int64>();
        msg->data = (element_++);
        RCLCPP_INFO(this->get_logger(), "Publishing num: %ld", msg->data);
        std::flush(std::cout);

        pub_->publish(std::move(msg));
    }
} // namespace example_comp

RCLCPP_COMPONENTS_REGISTER_NODE(example_comp::IntPub)