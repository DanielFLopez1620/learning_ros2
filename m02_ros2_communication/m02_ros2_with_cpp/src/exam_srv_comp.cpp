#include <iostream>
#include <memory>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "m02_ros2_with_cpp/srv/answer.hpp"
#include "m02_ros2_with_cpp/exam_srv_comp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace example_comp
{
    ExamServer::ExamServer(const rclcpp::NodeOptions & options)
    : Node("exam_srv", options)
    {
        auto handle_answer = 
        [this](
        const std::shared_ptr<m02_ros2_with_cpp::srv::Answer::Request> request,
        std::shared_ptr<m02_ros2_with_cpp::srv::Answer::Response> response
        ) -> void
        {
            RCLCPP_INFO(
                this->get_logger(), "For question of 4 answers, received [%"
                PRId32 "]" ,request->option);
            std::flush(std::cout);

            if (request->option == 3)
                response->correct = true;
            else
                response->correct = false;
        };

        srv_ = create_service<m02_ros2_with_cpp::srv::Answer>("exam_channel", 
            handle_answer);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(example_comp::ExamServer)