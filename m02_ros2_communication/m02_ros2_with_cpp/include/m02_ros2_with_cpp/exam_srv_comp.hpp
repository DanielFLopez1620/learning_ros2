#ifndef M02_ROS2_WITH_CPP__EXAM_SRV_COMP_HPP_
#define M02_ROS2_WITH_CPP__EXAM_SRV_COMP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "m02_ros2_with_cpp/srv/answer.hpp"
#include "m02_ros2_with_cpp/visibility_control.h"

namespace example_comp
{
    class ExamServer : public rclcpp::Node
    {
    public:
        M02_ROS2_WITH_CPP_PUBLIC
        explicit ExamServer(const rclcpp::NodeOptions & options);
    
    private:
        rclcpp::Service<m02_ros2_with_cpp::srv::Answer>::SharedPtr srv_;
    };
}

#endif // M02_ROS2_WITH_CPP__EXAM_SRV_COMP_HPP_