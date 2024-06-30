#ifndef M02_ROS2_WITH_CPP__EXAM_CLI_COMP_HPP_
#define M02_ROS2_WITH_CPP__EXAM_CLI_COMP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "m02_ros2_with_cpp/srv/answer.hpp"
#include "m02_ros2_with_cpp/visibility_control.h"

namespace example_comp
{
    class ExamClient : public rclcpp::Node
    {
    public:
        M02_ROS2_WITH_CPP_PUBLIC
        explicit ExamClient(const rclcpp::NodeOptions & options);
    
    protected:
        void on_timer();
    
    private:
        rclcpp::Client<m02_ros2_with_cpp::srv::Answer>::SharedPtr cli_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}

#endif // M02_ROS2_WITH_CPP__EXAM_CLI_COMP_HPP_