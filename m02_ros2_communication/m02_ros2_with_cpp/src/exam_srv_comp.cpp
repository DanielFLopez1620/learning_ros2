// --------------------------- REQUIRED STANDARD HEADERS ----------------------
#include <iostream>  // For I/O stream operations
#include <memory>    // For dynamic memory management
#include <cinttypes> // C header for format constant of printing

// --------------------------- ROS2 REQUIRED HEADERS --------------------------
#include "rclcpp/rclcpp.hpp"                          // For C++ with ROS2
#include "rclcpp_components/register_node_macro.hpp"  // Register component

// --------------------------- ROS2 SRV INTERFACE -----------------------------
#include "m02_ros2_with_cpp/srv/answer.hpp"   // For question like verification

// --------------------------- CUSTOM PACKAGES DEPENDENCIES ------------------
#include "m02_ros2_with_cpp/exam_srv_comp.hpp" // Server import prototype

// -------------------- COMPONENT IMPLEMENTATION IN NAMESPACE -----------------
namespace example_comp
{
    /**
     * Constructor for component configuration for the server that works as an
     * exam validator for question, in this case, it works as a verification for
     * a guess question, for a better usage check ExamSrv no component exmaple.
     * 
     * @param options Related with configs like intra_process_comm and others.
     */
    ExamServer::ExamServer(const rclcpp::NodeOptions & options)
    : Node("exam_srv", options)
    {
        // Create callback based on a lambda to check if the guess is correct.
        auto handle_answer = 
        [this](
        const std::shared_ptr<m02_ros2_with_cpp::srv::Answer::Request> request,
        std::shared_ptr<m02_ros2_with_cpp::srv::Answer::Response> response
        ) -> void
        {
            // Log message of answer/guess received
            RCLCPP_INFO(
                this->get_logger(), "For question of 4 answers, received [%"
                PRId32 "]" ,request->option);
            std::flush(std::cout);

            // Validate if option provided is the correct and respond
            if (request->option == 3)
                response->correct = true;
            else
                response->correct = false;
        };

        // Define server for server related with "exam_channel"
        srv_ = create_service<m02_ros2_with_cpp::srv::Answer>("exam_channel", 
            handle_answer);
    }
}

// Register component, acts like entry point to make the component discoverable
// to be loaded
RCLCPP_COMPONENTS_REGISTER_NODE(example_comp::ExamServer)