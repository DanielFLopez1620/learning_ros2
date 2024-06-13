// ------------------------------ STANDARARD HEADERS --------------------------

#include <chrono>         // For time usage with different precisions.
#include <cinttypes>      // For width base integral types
#include <cstdio>         // For standard input/output stream operations
#include <memory>         // Dynamic memory management
#include <string>         // Related with string usages
#include <utility>        // Variety of utilites from bit count to partial func
#include <bits/stdc++.h>  // Includes every std library

// ------------------------------ ROS2 RCLCPP REQUIRED ------------------------
#include "rclcpp/rclcpp.hpp"  // For C++ with ROS2

// -------------------------- ROS2 MSGS ---------------------------------------
#include "std_msgs/msg/float32.hpp"   // For uisng float of 32 bits


// ---------------------------- NAMESPACES CONSIDERED -------------------------
using namespace std::chrono_literals;

// ------------------------------ RANDOM PUBLISHER IMPLEMENTATION -------------
/**
 * Simple struct that contains a publisher of floating number that are pseudo-
 * randomly generated in a given topic.
 */
struct RandomPublisher : public rclcpp::Node
{
    /**
     * Constructor that initialize a node with a custom name that is capable
     * of intra communcation, as it thens creates a publisher that will later
     * use a unique_ptr mto share the message as we want to avoid coppies 
     * during the process.
     * 
     * @param name Name of the publisher node
     * @param output Topic/channel taht will be used for the process.
     */
    RandomPublisher(const std::string & name, const std::string & output)
    : Node(name,   // Iinitalize name by calling parent constructor 
           rclcpp::NodeOptions().use_intra_process_comms(true)) // Com. set up
    {
        // Create a publisher of float of 32 bits that uses the given topic 
        pub_ = this->create_publisher<std_msgs::msg::Float32>(output, 10);
        
        // Capture the publisher as weak_ptr to avoid keeping strong references
        // (so destruction is not avoided) and avoid references cycles for
        // memory leaks.s
        std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> 
            captured_pub = pub_;

        // Lambda function that will act as the callback of the publisher  
        auto callback = [captured_pub]() -> void
        {
            // Attempt to lock weak_ptr to get the shared_ptr 
            auto pub_ptr = captured_pub.lock();

            // Check if the publisher object still exists
            if(!pub_ptr)
            {   
                return;
            }

            // Pseudo-random number generation set up
            auto seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine gen(seed);
            std::uniform_real_distribution<float> distro(16.0, 20.0);

            // Instance message (by considering unique_ptr) and update its content
            std_msgs::msg::Float32::UniquePtr msg(new std_msgs::msg::Float32());
            msg->data = distro(gen);
            printf("Published pseudo random float: %f | Using address: 0x%" PRIXPTR "\n",
                msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
            
            // Publish message (by transferring ownership to prevent copies)
            pub_ptr->publish(std::move(msg));
        };
        
        // Create the timer and link the lambda (callback)
        timer_ = this->create_wall_timer(2.5s, callback);
    }

    // Define the publisher and timer as Shared pointers.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

}; // RandomPublisher

// ------------------- RANDOM SUBSCRIBER IMPLEMENTATION ------------------------
/**
 * Simple struct that contains a subscriber of floating numbers 32 bits that is
 * capable of mantaining intra communciation.
 */
struct RandomSubscriber : public rclcpp::Node
{
    /**
     * Constructor that uses the parent's interface to initlize a node with the
     * given name and subscriber to the given topic, then defines a subscriber
     * that will manage the intra communication process by considering a lambda
     * callback.
     */
    RandomSubscriber(const std::string & name, const std::string & input)
        : Node (name, // Node name
                rclcpp::NodeOptions().use_intra_process_comms(true))  // Com opt
    {
        // Instance a subscription for float32 msg by considering the given top
        sub_ = this->create_subscription<std_msgs::msg::Float32>(
            input,
            10,
            // Lambda definition for getting the message as unique_ptr (to
            // prevent copies) and the display what is received
            [](std_msgs::msg::Float32::UniquePtr msg)
            {
                printf("Received pseudo random float: %f | Using address: 0x%" 
                    PRIXPTR "\n", msg->data, 
                    reinterpret_cast<std::uintptr_t>(msg.get()));
            }
        );
    }

    // Subscription definiton
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;

}; // RandomSubscriber

int main(int argc, char* argv[])
{
    // Set buffering mode in this case for stdout stream (console), NULL (
    // buffer managed by itself), _IONBF (no buffering, output direct to
    // console) and BUFSIZ (ignored due to _IONBF) 
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize node and create executor
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor st_exec;
    
    // Define shared pointer toward nodes of interst
    auto intra_pub = std::make_shared<RandomPublisher>("rand_float_pub", "random_flt");
    auto intra_sub = std::make_shared<RandomPublisher>("rand_float_sub", "random_flt");

    // Link nodes to executor and spin
    st_exec.add_node(intra_pub);
    st_exec.add_node(intra_sub);
    st_exec.spin();

    // When everything is done, close the program
    rclcpp::shutdown();
    return 0;
}