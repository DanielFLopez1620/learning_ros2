/**
 * File: saying_hi.pp
 * 
 * Description: A simple node that prints the node saying hi with a name, that
 * can be modified by changing the value of the str param called 'your name'.
*/

// ------------------------------ CPP Standard Library -----------------------

#include <chrono>      // For flexible collection of types that tracks time with
                       // varying degrees of precision.

#include <functional>  // Function objects library, related with hash functions,
                       // those function with outputs that depends only on the
                       // input.

#include <string>      // Management and usage of strings.

// ------------------------------ rclcpp headers ------------------------------

#include <rclcpp/rclcpp.hpp> // Proper include for using rclcpp

// -------------------------- Implementation of namespaces --------------------
using namespace std::chrono_literals;

// ------------------------- Simple Displayer of Hi Class ---------------------

/**
 * Class that display a node saying hi and appending a name linked to a param,
 * that can be modified in the terminal or by other node.
*/
class SayingHi : public rclcpp::Node
{
  // Public interface
  public:
    /**
     * Constructo call that initialize a node with a given name (using parent 
     * interface), instance a parameter (with its own descriptor), and instnace
     * a timer with a fixed rate linked to a callback for pinting the info.
    */
    SayingHi() : Node("saying_your_name")
    {
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description = 
        "Put your name here, so you will be called by it!";
      this->declare_parameter("your_name", "ROS User");

      timer_ = this->create_wall_timer(
        5000ms, std::bind(&SayingHi::say_callback, this));
    }

    /**
     * Callback that will be used when the timer indicate to print a new message
     * on the terminal.
    */
    void say_callback()
    {
      // Get param value
      std::string change_param = this->get_parameter("your_name").as_string();

      // Display info
      RCLCPP_INFO(this->get_logger(), "Hi! I am a node, and you are %s.",
        change_param.c_str());

      // Update the param value with the original one
      std::vector<rclcpp::Parameter> new_params{
        rclcpp::Parameter("your_name", "ROS User")};
      this->set_parameters(new_params);
    }

  private:
    // Instance a timer object using a shared pointer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  // Initialize ROS Client Library for CPP
  rclcpp::init(argc, argv);

  // Spin (loop) for multiple prints
  rclcpp::spin(std::make_shared<SayingHi>());

  // CLose and shutdown before final return
  rclcpp::shutdown();
  return 0;
}