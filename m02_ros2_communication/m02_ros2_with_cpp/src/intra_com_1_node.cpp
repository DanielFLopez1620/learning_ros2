#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/char.hpp"

using namespace std::chrono_literals;

struct IncrementerASCII : public rclcpp::Node
{
    IncrementerASCII(const std::string & name, const std::string & in, 
        const std::string & out)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        pub = this->create_publisher<std_msgs::msg::Char>(out, 10);
        std::weak_ptr<std::remove_pointer<decltype(pub.get())>::type> captured_pub = pub;

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
                if(!rclcpp::sleep_for(2s))
                {
                    return;
                }
                int ascii = (int) msg->data;
                
                if( ++ascii > 126 )
                {
                    ascii = 33;
                }
                msg->data = (char)(ascii);
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
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor st_exec;

    auto ascii1 = std::make_shared<IncrementerASCII>("first_ascii_node", 
        "channel1", "channel2");
    auto ascii2 = std::make_shared<IncrementerASCII>("second_ascii_node",
        "channel2", "channel1");
    
    rclcpp::sleep_for(2s);
    std::unique_ptr<std_msgs::msg::Char> msg(new std_msgs::msg::Char());
    msg->data = '!';

    printf("First (and only manual publication) letter: %c | Memory Address: 0x%" 
        PRIXPTR "\n", msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
    ascii1->pub->publish(std::move(msg));

    st_exec.add_node(ascii1);
    st_exec.add_node(ascii2);
    st_exec.spin();

    rclcpp::shutdown();
    return 0;
}