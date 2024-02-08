/**
 * File: exam_srv.cpp
 * 
 * Description: Simple server for getting the answers of a one question exam
 * and indicate to the client, if the answer provided was right, the 
 * implementation was made with ROS2.
*/

// ---------------------------- CPP Standard Libraries -----------------------

#include <memory> // General utitilites to manage dynamic memory

// -------------------------------- rclcpp Headers ---------------------------

#include "rclcpp/rclcpp.hpp"  // Proper include for using rclcpp

#include "m02_ros2_with_cpp/srv/answer.hpp" // Include srv file 


// -------------------------------- Function Prototypes -----------------------

void check(const std::shared_ptr<m02_ros2_with_cpp::srv::Answer::Request> 
            request,
         std::shared_ptr<m02_ros2_with_cpp::srv::Answer::Response>
            response);

// ------------------------------- MAIN PROGRAM -------------------------------
int main(int argc, char **argv)
{
  // ROS CLient Library for C++ initialization
  rclcpp::init(argc, argv);

  // Initialize node called 'add_two_ints_server' by defining a shared pointer
  std::shared_ptr<rclcpp::Node> node = 
    rclcpp::Node::make_shared("exam_srv");

  // Instance a service server object that hear to request of AddTwoInts by the 
  // service 'add_two_ints' and linking a callback for responses.
  rclcpp::Service<m02_ros2_with_cpp::srv::Answer>::SharedPtr service =
    node->create_service<m02_ros2_with_cpp::srv::Answer>("exam_channel", 
                                                              &check);
  // Log that service is ready to attend
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to evaluate...\n");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "What distro are we using?\n");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "a) Noetic\n");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "b) Foxy\n");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "c) Humble\n");

  // Spin node to manage multiple requests
  rclcpp::spin(node);

  // Close node and shutdown library usage
  rclcpp::shutdown();
}

// -------------------------------- Function Definitions ----------------------

/**
 * Check callback that will manage incoming requests that provide an answer and
 * evaluate them returning a feedback
 * 
 * @param request Pointer to the request (answer) provided by client
 * @param response Pointer to response with the feedback of the answer.
*/
void check(const std::shared_ptr<m02_ros2_with_cpp::srv::Answer::Request> 
            request,
         std::shared_ptr<m02_ros2_with_cpp::srv::Answer::Response>
            response)
{
  auto correct_ans = 'c';
  // Update response by making the operation
  response->correct = (correct_ans == request->option) ? true : false;

  // Log info of the process
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Answer received: %c", 
              request->option);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending feeback: [%s]", 
              (response->correct) ? "Correct" : "Wrong");
}