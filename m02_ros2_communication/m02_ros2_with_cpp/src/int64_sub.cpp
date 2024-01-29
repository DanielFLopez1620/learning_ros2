/**
 * File: int64_sub.cpp
 * 
 * Description: Simple subscriber for integer (64 bits) implemented with the 
 * rclpp.
*/

// ------------------------- CPP standard Libraries --------------------------

#include <memory>  // General utilities to manage dynamic memory

// ------------------------ rclcpp headers -----------------------------------

#include "rclcpp/rclcpp.hpp"       // Proper include for using rclcpp

#include "std_msgs/msg/int64.hpp"  // Include of standard message for int64

// ------------------------ Implementations of namespaces --------------------

// Namespace for std::bind place holders
using std::placeholders::_1;


// ------------------------- Int64 Subscriber Class --------------------------

/**
 * Simple class for a subscriber of integers (64 bits) which inherits from the 
 * Node class
*/
class IntSub : public rclcpp::Node
{
  // Public interface
  public:
    /**
     * Constructor call that initialize the node name (using parent interface),
     * intance a subscriber object for integers by usin the 'num_int64' topic
     * and linking a callback for displaying the message received.
     * 
    */
    IntSub() : Node("int64_sub")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "num_int64", 10, std::bind(&IntSub::num_callback, this, _1));
    }

  // Private interface
  private:
    /**
     * Callback that will print the message received.
     * 
     * @param msg Pointer to the message received
    */
    void num_callback(const std_msgs::msg::Int64 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Message received: '%ld'", msg.data);
    }

    // Private shared pointer for subscriber instance
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // ROS Client Library for C++ initialization
  rclcpp::init(argc, argv);

  // Spin until message is sent
  rclcpp::spin(std::make_shared<IntSub>());

  // Closing node
  rclcpp::shutdown();
  return 0;
}