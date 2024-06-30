#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "m02_ros2_with_cpp/exam_cli_comp.hpp"
#include "m02_ros2_with_cpp/srv/answer.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace example_comp
{
    ExamClient::ExamClient(const rclcpp::NodeOptions & options)
    : Node("exam_cli", options)
    {
        cli_ = create_client<m02_ros2_with_cpp::srv::Answer>("exam_channel");

        timer_ = create_wall_timer(5s, std::bind(&ExamClient::on_timer, this));
    }

    void ExamClient::on_timer()
    {
        if (!cli_->wait_for_service(2s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Service and ROS System failing or manually interrupted..."
                );
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Service isn't available");
            return;
        }

        auto request = std::make_shared<m02_ros2_with_cpp::srv::Answer::Request>();
        request->option = 3;

        using SrvResponseFuture = 
            rclcpp::Client<m02_ros2_with_cpp::srv::Answer>::SharedFuture;
        auto response_link_callback = [this](SrvResponseFuture future)
        {
            RCLCPP_INFO(this->get_logger(), "Feedback of answer received: %d",
                future.get()->correct);
        };

        auto future_result = cli_->async_send_request(request, 
            response_link_callback);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(example_comp::ExamClient)