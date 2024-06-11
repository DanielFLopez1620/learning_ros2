#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

struct RandomPublisher : public rclcpp::Node
{
    RandomPublisher(const std::string & name, const std::string & output)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        pub_ = this->create_publisher<std_msgs::msg::Float32>(output, 10);
        std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> 
            captured_pub = pub_;
        auto callback = [captured_pub]() -> void {
            auto pub_ptr = captured_pub.lock();
            if(!pub_ptr)
            {   
                return;
            }

        }
        timer_ = this->create_wall_timer(2.5s, callback);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
};