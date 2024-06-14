#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msgs/char.hpp"

using namespace std::chrono_literals;

struct IncrementerASCII : public rclcpp::Node
{
    IncrementerASCII(const std::string & name, const std::string & in, 
        const std::string & out)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        pub = this->create_publisher<std_msgs::msg::Char>(out, 10);
        std::weak_ptr<std::remove_pointer<decltype(pub.get())>::type> capture_pub = pub;

        sub = this->create_subscription<std_msgs::msg::Char>
        (
            in,
            10,
            [captured_pub](std_msgs::msg::Char::UniquePtr msg)
            {
                auto pub_ptr = captured_pub.lock();
                if(!pub_ptr)
                {
                    return;
                }
                printf("Letter received: %c | Memory address: 0x%" PRIXPTR "\n",
                    msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
                if(!rclcpp::sleep_for(1.5s))
                {
                    return;
                }
                int ascii = (int msg->data)
                
                if( ++ascii > 126 )
                {
                    ascii = 33;
                }
                msg->data = (char)(ascii)
                printf("Incrementing ASCII Value: %c | Memory address: 0x%" PRIXPTR "\n",
                    msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
                pub_ptr->publish(std::move(msg));
            }
        );
    }

    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr pub;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr sub;
};

int main(int argc, char* argv[])
{
    
    return 0;
}