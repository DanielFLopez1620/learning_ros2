/**
 * File: add_two_nums_srv.cpp
 * 
 * Description: Simple service for adding two nums with the rclcpp
 * implementation for ROS2.
*/

// ---------------------------- CPP Standard Libraries -----------------------

#include <memory> // General utitilites to manage dynamic memory

// -------------------------------- rclcpp Headers ---------------------------

#include "rclcpp/rclcpp.hpp"  // Proper include for using rclcpp

#include "example_interfaces/srv/add_two_ints.hpp" // Include srv file 


// -------------------------------- Function Prototypes -----------------------

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> 
            request,
         std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>
            response);

// ------------------------------- MAIN PROGRAM -------------------------------
int main(int argc, char **argv)
{
  // ROS CLient Library for C++ initialization
  rclcpp::init(argc, argv);

  // Initialize node called 'add_two_ints_server' by defining a shared pointer
  std::shared_ptr<rclcpp::Node> node = 
    rclcpp::Node::make_shared("add_two_ints_server");

  // Instance a service server object that hear to request of AddTwoInts by the 
  // service 'add_two_ints' and linking a callback for responses.
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", 
                                                              &add);
  // Log that service is ready to attend
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  // Spin node to manage multiple requests
  rclcpp::spin(node);

  // Close node and shutdown library usage
  rclcpp::shutdown();
}

// -------------------------------- Function Definitions ----------------------

/**
 * Add callback that will manage incoming requests of two integers two give the
 * proper response and send it.
 * 
 * @param request Pointer to the request provided by client
 * @param response Pointer to response that ig going to be sent to the client.
*/
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> 
           request,
         std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>
           response)
{
  // Update response by making the operation
  response->sum = request->a + request->b;

  // Log info of the process
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Request received \na: %ld  b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sendind response: [%ld]", 
              (long int)response->sum);
}