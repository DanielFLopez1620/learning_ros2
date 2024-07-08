#ifndef M02_ROS2_WITH_CPP__INT64_SUB_COMP_HPP_
#define M02_ROS2_WITH_CPP__INT64_SUB_COMP_HPP_

// ------------------------ ROS2 REQUIRED HEADERS -----------------------------
#include "rclcpp/rclcpp.hpp"  // For C++ with ROS2

// ------------------------ ROS2 MESSAGES INTERFACES -------------------------
#include "std_msgs/msg/int64.hpp" // Int64 interface

// ------------------------ CUSTOM PACKAGE DEPENDENCIES ----------------------
#include "m02_ros2_with_cpp/visibility_control.h"  // Compilation visibility

// -------------------- COMPONENT IMPLEMENTATION IN NAMESPACE -----------------
namespace example_comp
{
    /**
     * Class oriented to create a component for subscription of int64 intergers
     * that can be configured with node options.
     */
    class IntSub : public rclcpp::Node
    {
    public:
        M02_ROS2_WITH_CPP_PUBLIC

        // Constructor prototype that must be implemented
        explicit IntSub(const rclcpp::NodeOptions & options);
    
    private:
        // Subscriptor declaration
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_;

    }; // class IntSub

} // namespace example_com

#endif  // M02_ROS2_WITH_CPP__INT64_SUB_COMP_HPP_
