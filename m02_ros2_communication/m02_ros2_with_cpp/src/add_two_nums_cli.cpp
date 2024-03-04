/**
 * File: add_two_nums_srv.cpp
 * 
 * Description: Simple client for adding two nums with the rclcpp 
 * implementation for ROS2
*/

// --------------------------- CPP Standard Libraries -------------------------

#include <chrono>  // For flexible collecction of types that tracks time with
                   // varying types of precision.

#include <cstdlib> // General purpose function

#include <memory>  // General utilities to manage dynamic memory

// --------------------------- rclcpp Headers ---------------------------------

#include "rclcpp/rclcpp.hpp"   // Proper include for using rclcpp

#include "example_interfaces/srv/add_two_ints.hpp"  // AddTwoInts service

// --------------------------- Implementation of namespaces -------------------
using namespace std::chrono_literals;

// --------------------------- MAIN PROGRAM -----------------------------------
int main(int argc, char **argv)
{
  // ROS CLient Library for C++ initialization
  rclcpp::init(argc, argv);

  // Check that are enough arguments "node + operands (2)"
  if (argc != 3) 
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
    return 1;
  }

  // Node init with name "add_two_ints_client" by defining a shared pointer
  std::shared_ptr<rclcpp::Node> node = 
    rclcpp::Node::make_shared("add_two_ints_client");

  // Instance a service client objet that sends request of AddTwoInts by the
  // service 'add_two_ints'.
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  // Instance a request object and assign the two numbers as summands, atoll is
  // needed to make the conversion from C-String to long long int.
  auto request = 
    std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  // Loop for waiting to the server to be online (logs message each second)
  while (!client->wait_for_service(1s)) 
  {
    // Exits the program if something goes wrong
    if (!rclcpp::ok()) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                "service not available, waiting again...");
  }

  // After connection is made, receive the asynchronous request
  auto result = client->async_send_request(request);

  // Wait for the result and check if the value is correctly received
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                 "Failed to call service add_two_ints");
  }

  // Close after printing result
  rclcpp::shutdown();
  return 0;
}