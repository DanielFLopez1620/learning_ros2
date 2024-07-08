// ------------------------ STANDARD REQUIRED HEADERS -------------------------
#include <iostream>  // For I/O stream operations
#include <memory>    // Dynamic memory management

// ------------------------ ROS2 REQUIRED HEADERS -----------------------------
#include "rclcpp/rclcpp.hpp"                            // For C++ with ROS2
#include "rclcpp_components/register_node_macro.hpp"    // Component register

// ------------------------ ROS2 MESSAGES INTERFACES --------------------------
#include "std_msgs/msg/int64.hpp"  // Int64 interface

// ------------------------ CUSTOM PACKAGES DEPENDENCIES ----------------------
#include "m02_ros2_with_cpp/int64_sub_comp.hpp" // Header of the component def

// -------------------- COMPONENT IMPLEMENTATION IN NAMESPACE -----------------
namespace example_comp
{
    /**
     * A class constructor oriented to further setup and configure a subscriber
     * for integers.
     * 
     * @param options Related with configurations of intra_process_com and oth.
     */
    IntSub::IntSub(const rclcpp::NodeOptions & options)
    : Node("int64_sub", options)
    {
        // Callback definition as a lambda that considers the class itself and 
        // the message received as a shared pointer for logging the content.
        // If you want, you can create it with UniquePtr for zero-copy transport
        auto callback =
            [this](std_msgs::msg::Int64::ConstSharedPtr msg) -> void
            {
                // Log received messages
                RCLCPP_INFO(this->get_logger(), "Reading num: %ld", msg->data);
                std::flush(std::cout);
            };
        
        // Definition of a subscription for integers in num_int64 topic.
        // Not ethat not every publisher on the same topic will be compatible as they
        // must have compatible Quality of Service policies
        sub_ = create_subscription<std_msgs::msg::Int64>("num_int64", 10, callback);
    } // IntSub::IntSub
}

// Adding component
RCLCPP_COMPONENTS_REGISTER_NODE(example_comp::IntSub)
