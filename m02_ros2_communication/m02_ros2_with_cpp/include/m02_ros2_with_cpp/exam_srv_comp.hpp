#ifndef M02_ROS2_WITH_CPP__EXAM_SRV_COMP_HPP_
#define M02_ROS2_WITH_CPP__EXAM_SRV_COMP_HPP_

// --------------------------- ROS2 REQUIRED HEADERS -------------------------
#include "rclcpp/rclcpp.hpp"  // For ROS2 with C++

// --------------------------- ROS2 SRV INTERFACES ---------------------------
#include "m02_ros2_with_cpp/srv/answer.hpp"  // srv for question exam answer

// -------------------------- CUSTOM PACKAGES DEPENDENCIES -------------------
#include "m02_ros2_with_cpp/visibility_control.h" // Compilation related

// -------------------- COMPONENT IMPLEMENTATION IN NAMESPACE -----------------
namespace example_comp
{
    /**
     * Class oriented to evaluate request related with an exam question, in this
     * case it is indeterminated, but if you want a more contextualized example
     * check the no component implementation of the ExamSrv.
     */
    class ExamServer : public rclcpp::Node
    {
    public:
        M02_ROS2_WITH_CPP_PUBLIC
        // Prototype constructor that must be implemented later.
        explicit ExamServer(const rclcpp::NodeOptions & options);
    
    private:
        // Declaration of the server
        rclcpp::Service<m02_ros2_with_cpp::srv::Answer>::SharedPtr srv_;
    };
}

#endif // M02_ROS2_WITH_CPP__EXAM_SRV_COMP_HPP_