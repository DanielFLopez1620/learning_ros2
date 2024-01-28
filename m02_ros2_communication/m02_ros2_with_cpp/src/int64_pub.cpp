/**
 * File: int64_pub.cpp
 * 
 * Description: Simple publisher for integer (64 bits) implemented with the 
 * rclcpp implementation for ROS.
*/

// ------------------------- CPP standard Libraries --------------------------

#include <chrono>     // For flexible collection of types that trask time with
                      // varying degrees of precision.

#include <functional> // Function objects library, related with hash functions,
                      // those function with outputs that depends only on the
                      // input.

#include <memory>     // General utilities to manage dynamic memory

#include <string>     // Management and usage of strings.

#include <cstdlib>    // General purpose functions 

// ------------------------ rclcpp headers -----------------------------------

#include "rclcpp/rclcpp.hpp"        // Proper include for using rclcpp 

#include "std_msgs/msg/int64.hpp"   // Include of standard message for int64

// ------------------------ Implementations of namespaces --------------------
using namespace std::chrono_literals;


// ------------------------- Int64 Publisher Class ---------------------------

/**
 * Simple class for a publisher of integers (64 bits) which inherits from Node
 * class.
*/
class IntPub : public rclcpp::Node
{
  // Public interface
  public:
    /**
     * Constructor call that initialize the node name (using parent interface),
     * instance a publisher object for integers by using the 'num_int64'
     * topic and linking a callback with a timer.
    */
    IntPub() : Node("int64_pub"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Int64>("num_int64", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&IntPub::timer_callback, this));
    }
  
  // Private interface
  private:
    /**
     * Method for publishing the integer when the established time for the
     * timer is completed.
    */
    void timer_callback()
    {
      // Defining lower and upper limit for random
      auto ll = 1, ul = 100;

      // Instance standard message obj for integers
      auto message = std_msgs::msg::Int64();

      // Updating content with random number
      message.data = rand() % (ul - ll + 1) + ll;

      // Log and publish info
      RCLCPP_INFO(this->get_logger(), "Publishing (%d): %d", count_ ,message.data);
      publisher_->publish(message);
    }

    // Private shared pointer for timer instance
    rclcpp::TimerBase::SharedPtr timer_;

    // Private shared pointer for publisher instance
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    
    // Counter
    size_t count_;
};



int main(int argc, char * argv[])
{
  // ROS Client Library for C++ initialization
  rclcpp::init(argc, argv);

  // Spin the publisher 
  rclcpp::spin(std::make_shared<IntPub>());

  // Closing node
  rclcpp::shutdown();
  return 0;
}