#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <bits/stdc++.h>

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
        auto callback = [captured_pub]() -> void
        {
            
            auto pub_ptr = captured_pub.lock();
            if(!pub_ptr)
            {   
                return;
            }

            auto seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine gen(seed);
            std::uniform_real_distribution<float> distro(16.0, 20.0);
            std_msgs::msg::Float32::UniquePtr msg(new std_msgs::msg::Float32());
            msg->data = distro(gen);
            printf("Published pseudo random float: %f | Using address: 0x%" PRIXPTR "\n",
                msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
            
            pub_ptr->publish(std::move(msg));
        };
        
        timer_ = this->create_wall_timer(2.5s, callback);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

struct RandomSubscriber : public rclcpp::Node
{
    RandomSubscriber(const std::string & name, const std::string & input)
        : Node (name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        sub_ = this->create_subscription<std_msgs::msg::Float32>(
            input,
            10,
            [](std_msgs::msg::Float32::UniquePtr msg)
            {
                printf("Received pseudo random float: %f | Using address: 0x%" 
                    PRIXPTR "\n", msg->data, 
                    reinterpret_cast<std::uintptr_t>(msg.get()));
            }
        );
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
};

int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor st_exec;
    
    auto intra_pub = std::make_shared<RandomPublisher>("rand_float_pub", "random_flt");
    auto intra_sub = std::make_shared<RandomPublisher>("rand_float_sub", "random_flt");

    st_exec.add_node(intra_pub);
    st_exec.add_node(intra_sub);
    st_exec.spin();

    rclcpp::shutdown();

    return 0;
}