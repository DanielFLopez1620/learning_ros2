#ifndef M02_ROS2_WITH_CPP__INT64_SUB_COMP_HPP_
#define M02_ROS2_WITH_CPP__INT64_SUB_COMP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "m02_ros2_with_cpp/visibility_control.h"

namespace example_comp
{
    class IntSub : public rclcpp::Node
    {
    public:
        M02_ROS2_WITH_CPP_PUBLIC
        explicit IntSub(const rclcpp::NodeOptions & options);
    
    private:
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_;

    };
}

#endif  // M02_ROS2_WITH_CPP__INT64_SUB_COMP_HPP_
