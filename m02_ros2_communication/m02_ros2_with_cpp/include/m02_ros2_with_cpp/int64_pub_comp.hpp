#ifndef M02_ROS2_WITH_CPP__INT64_PUB_COMP_
#define M02_ROS2_WITH_CPP__INT64_PUB_COMP_

// ----------------------- ROS2 REQUIRED HEADERS ------------------------------
#include "rclcpp/rclcpp.hpp"  // For C++ with ROS2

// ----------------------- ROS2 MESSAGES INTERFACES ---------------------------
#include "std_msgs/msg/int64.hpp" // Int64 interface

// ------------------------ CUSTOM PACKAGES HEADER ----------------------------
#include "m02_ros2_with_cpp/visibility_control.h"  // Compilation visibility

// -------------------- COMPONENT IMPLEMENTATION IN NAMESPACE -----------------
namespace example_comp
{
    /**
     * Class oriented to develop a component for a simple publisher that can be
     * configured and optimized.
     */
    class IntPub : public rclcpp::Node
    {
    public:
        M02_ROS2_WITH_CPP_PUBLIC
        // Prototype for constructor that must be implemented
        explicit IntPub(const rclcpp::NodeOptions & options);

    protected:
        // Prototype for timer callback implementation
        void on_timer();
        
    private:
        // Integer element that will have a one increment when timer updates
        int element_;

        // Declaration of publisher and timer for future definitions
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

    }; // class IntPub

} // namespace example_comp


#endif  // M02_ROS2_WITH_CPP__INT64_PUB_COMP_