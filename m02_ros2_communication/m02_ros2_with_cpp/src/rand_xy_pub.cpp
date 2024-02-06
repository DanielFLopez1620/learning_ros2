/**
 * File: int64_pub.cpp
 * 
 * Description: Simple publisher for integer (64 bits) implemented with the 
 * rclcpp implementation for ROS.
*/

// ------------------------- CPP standard Libraries --------------------------

#include <chrono>     // For flexible collection of types that tracks time with
                      // varying degrees of precision.

#include <functional> // Function objects library, related with hash functions,
                      // those function with outputs that depends only on the
                      // input.

#include <memory>     // General utilities to manage dynamic memory

#include <string>     // Management and usage of strings.

#include <cstdlib>    // General purpose functions 

#include <ctime>      // Library for time usage 

// ------------------------ rclcpp headers -----------------------------------

#include "rclcpp/rclcpp.hpp"    // Proper include for using rclcpp 

#include "m02_ros2_with_cpp/msg/rare_point.hpp"   // Include custom msg

// ------------------------ Implementations of namespaces --------------------
using namespace std::chrono_literals;


// ------------------------- Int64 Publisher Class ---------------------------

/**
 * Simple class for a publisher of RarePoints (X position integer and Y 
 * position floating numbers), linked to a timer.
*/
class RandomXYPub : public rclcpp::Node
{
  // Public interface
  public:
    /**
     * Constructor call that initialize the node name (using parent interface),
     * instance a publisher object for cuestom "rare point" by using the 
     * "rare_point" topic and linking a callback with a fixed timer.
    */
    RandomXYPub() : Node("random_xy_pub"), count_(0)
    {
      publisher_ = 
        this->create_publisher<m02_ros2_with_cpp::msg::RarePoint>(
          "rare_point", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&RandomXYPub::timer_callback, this));
    }
  
  // Private interface
  private:
    /**
     * Method for publishing the rare point (integer X, float Y) when the 
     * established time for the timer is completed.
    */
    void timer_callback()
    {
      // Adding seed for non fixed random numbers
      srand((unsigned int) time(NULL));
      
      // Defining lower and upper limit for random
      auto ll = 1, ul = 100;

      // Instance standard message obj for integers
      auto message = m02_ros2_with_cpp::msg::RarePoint();

      // Updating content with random number
      message.x = rand() % (ul - ll + 1) + ll;
      message.y = float(rand())/float((RAND_MAX)) * message.x;

      // Log and publish info
      RCLCPP_INFO(this->get_logger(), "Rare point # (%ld): (%ld, %f)", 
                  count_ ,message.x , message.y);
      publisher_->publish(message);

      this->count_++;

    }

    // Private shared pointer for timer instance
    rclcpp::TimerBase::SharedPtr timer_;

    // Private shared pointer for publisher instance
    rclcpp::Publisher<m02_ros2_with_cpp::msg::RarePoint>::SharedPtr publisher_;
    
    // Counter
    size_t count_;
};

// ----------------------------- MAIN PROGRAM ---------------------------------
int main(int argc, char * argv[])
{
  // ROS Client Library for C++ initialization
  rclcpp::init(argc, argv);

  // Spin the publisher 
  rclcpp::spin(std::make_shared<RandomXYPub>());

  // Closing node
  rclcpp::shutdown();
  return 0;
}