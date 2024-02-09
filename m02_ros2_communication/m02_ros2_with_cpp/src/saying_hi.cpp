#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class SayingHi : public rclcpp::Node
{
  public:
    SayingHi() : Node("minimal_param_node")
    {
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description = "Put your name here, so you will be called by it!";
      this->declare_parameter("your_name", "ROS User");

      timer_ = this->create_wall_timer(
        1000ms, std::bind(&SayingHi::say_callback, this));
    }

    void say_callback()
    {
      std::string change_param = this->get_parameter("your_name").as_string();

      RCLCPP_INFO(this->get_logger(), "Hi! I am a node, and you are %s.",
        change_param.c_str());

      std::vector<rclcpp::Parameter> new_params{
        rclcpp::Parameter("your_name", "ROS User")};
      
      this->set_parameter(new_params);
      
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SayingHi>());
  rclcpp::shutdown();
  return 0;
}