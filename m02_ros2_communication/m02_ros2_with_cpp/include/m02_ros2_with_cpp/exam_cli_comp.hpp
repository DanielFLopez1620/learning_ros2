#ifndef M02_ROS2_WITH_CPP__EXAM_CLI_COMP_HPP_
#define M02_ROS2_WITH_CPP__EXAM_CLI_COMP_HPP_

// ----------------- ROS2 REQUIRED HEADERS ------------------------------------
#include "rclcpp/rclcpp.hpp" 

// ------------------ ROS2 SERVICE INTERFACE ---------------------------------
#include "m02_ros2_with_cpp/srv/answer.hpp"  // Answer server for quesitons

// ------------------ CUSTOM PACKAGE DEPENDENCIES -----------------------------
#include "m02_ros2_with_cpp/visibility_control.h" // Related with compilation

// -------------------- COMPONENT IMPLEMENTATION IN NAMESPACE -----------------
namespace example_comp
{
    /**
     * Class oriented to send a value for a unspecified question of multiple
     * options where one have to be selected (for better understanding of the
     * service check the no component implementation). 
     */
    class ExamClient : public rclcpp::Node
    {
    public:
        M02_ROS2_WITH_CPP_PUBLIC
        // Constructor portotype that must be implemented later
        explicit ExamClient(const rclcpp::NodeOptions & options);
    
    protected:
        // Prototype for timer callback
        void on_timer();
    
    private:
        // Declaration of the client and timer
        rclcpp::Client<m02_ros2_with_cpp::srv::Answer>::SharedPtr cli_;
        rclcpp::TimerBase::SharedPtr timer_;

    }; // class ExamClient

} // namespace example_comp

#endif // M02_ROS2_WITH_CPP__EXAM_CLI_COMP_HPP_