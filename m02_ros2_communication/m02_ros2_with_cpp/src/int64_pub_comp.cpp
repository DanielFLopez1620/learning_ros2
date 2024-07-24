// ----------------------- STANDARD HEADERS REQUIRED --------------------------
#include <chrono>    //  Related with time operations in different precisions
#include <iostream>  // For I/O stream operations
#include <memory>    // Dynamic memory management
#include <utility>   // Variety of utilities from bit-counting to partial funcs

// ----------------------- RCLCPP REQUIRED ------------------------------------ 
#include "rclcpp/rclcpp.hpp"                           // For C++ with ROS2
#include "rclcpp_components/register_node_macro.hpp"   // Component register

// ---------------------- ROS2 INTERFACES REQUIRED ----------------------------
#include "std_msgs/msg/int64.hpp"  // Int64 interface

// ---------------------- CUSTOM PACKAGE DEPENDENCIES -------------------------
#include "m02_ros2_with_cpp/int64_pub_comp.hpp"  // Header of the component def

// ---------------------- NAMESPACES USED -------------------------------------
using namespace std::chrono_literals;

// -------------------- COMPONENT IMPLEMENTATION IN NAMESPACE -----------------
namespace example_comp
{
    /**
     * Interger publisher class constructor for usage as a component that allows 
     * further configuration and optimization, while it creates a publisher and
     * a timer.
     * 
     * @param options Node configuration for intra_process_comm and related.
     */
    IntPub::IntPub(const rclcpp::NodeOptions & options)
    : Node("int64_pub_comp", options), element_(0)
    {
        // Definition of integer publisher on 'num_int64' topic.
        pub_ = create_publisher<std_msgs::msg::Int64>("num_int64",10);

        // Timer required for periodic message sending operations.
        timer_ = create_wall_timer(500ms, std::bind(&IntPub::on_timer, this));
    
    } // IntPub::IntPub

    /**
     * When timer's call is made, it publishes a incremented integer each time.
     * 
     */
    void IntPub::on_timer()
    {
        // Instance message using unique pointer for avoiding copies and
        // better optimization.
        auto msg = std::make_unique<std_msgs::msg::Int64>();

        // Add message content by considering an increment on stored value
        msg->data = (element_++);

        // Log process
        RCLCPP_INFO(this->get_logger(), "Publishing num: %ld", msg->data);
        std::flush(std::cout);

        // Publish message (by moving content to avoid copies)
        pub_->publish(std::move(msg));
    
    } // IntPub::on_timer()

} // namespace example_comp

// Adding component
RCLCPP_COMPONENTS_REGISTER_NODE(example_comp::IntPub)