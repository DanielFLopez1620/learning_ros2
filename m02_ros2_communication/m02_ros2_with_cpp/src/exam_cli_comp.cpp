// ------------------------ STANDARD REQUIRED HEADERS -------------------------
#include <iostream>  // For I/O stream operations
#include <memory>    // For dynamic memory management

// --------------------- ROS2 REQUIRED HEADERS --------------------------------
#include "rclcpp/rclcpp.hpp"                           // For C++ with ROS2
#include "rclcpp_components/register_node_macro.hpp"   // Component register

// --------------------- ROS2 SRV INTERFACES ----------------------------------
#include "m02_ros2_with_cpp/srv/answer.hpp" // Exam question service

// ---------------------- CUSTOM PACKAGE DEPENDENCIES -------------------------
#include "m02_ros2_with_cpp/exam_cli_comp.hpp" 

// ---------------------- NAMESPACE TO CONSIDER -------------------------------
using namespace std::chrono_literals;

// -------------------- COMPONENT IMPLEMENTATION IN NAMESPACE -----------------
namespace example_comp
{
    /**
     * Constructor oriented to configure the node options for component usage,
     * instance the client and a timer to send request by linking a callback.
     * 
     * @param options Configurations related with intra_process_com and others
     */
    ExamClient::ExamClient(const rclcpp::NodeOptions & options)
    : Node("exam_cli", options)
    {
        // Client definition for using service 'exam_channel'.
        cli_ = create_client<m02_ros2_with_cpp::srv::Answer>("exam_channel");

        // Timer defintion, should consider a period greater than the duration
        // of the callback execution to prevent thread problems.
        timer_ = create_wall_timer(5s, std::bind(&ExamClient::on_timer, this));
    }

    /**
     * Timer callbac for sending client request to the server, in this case, we
     * send answers to a multiple option question and the server have to tell us
     * if it was the correct option (here we do not have interaction or a question
     * then it is just a component example with custom interfaces)
     */
    void ExamClient::on_timer()
    {
        // Checking if service is available
        if (!cli_->wait_for_service(2s))
        {
            // Also checking that ROS2 communications is working
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

        // Instance proper request and create value
        auto request = std::make_shared<m02_ros2_with_cpp::srv::Answer::Request>();
        request->option = 3;

        // Define future response for asynchronous process
        using SrvResponseFuture = 
            rclcpp::Client<m02_ros2_with_cpp::srv::Answer>::SharedFuture;
        
        // Create a lambda to display the result
        auto response_link_callback = [this](SrvResponseFuture future)
        {
            RCLCPP_INFO(this->get_logger(), "Feedback of answer received: %d",
                future.get()->correct);
        };

        // Link the asynchronous respond with the lambda
        auto future_result = cli_->async_send_request(request, 
            response_link_callback);
    }
}

// Register component
RCLCPP_COMPONENTS_REGISTER_NODE(example_comp::ExamClient)