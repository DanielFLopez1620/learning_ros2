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
class MinimalSubscriber : public rclcpp::Node
{
  // Public interface
  public:
    /**
     * Constructor call that initialize the node name (using parent interface),
     * intance a subscriber object for integers by usin the 'num_int64' topic
     * and linking a callback for displaying the message received.
     * 
    */
    MinimalSubscriber()
    : Node("int64_sub")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}