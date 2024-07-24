// ----------------------- STANDARD HEADERS -----------------------------------
#include <chrono>     // For time usage with different precisions.
#include <cinttypes>  // For width base integral types
#include <cstdio>     // For standard input/ouput stream operations
#include <memory>     // Dynamic memory management
#include <string>     // Related with string usage
#include <utility>    // Variety of utilities from bit count to partial funcs

// -----------------------  ROS2 RCLCPP REQUIRED ------------------------------
#include "rclcpp/rclcpp.hpp"  // For C++ with ROS2

// ----------------------- ROS2 MSG REQUIRED ----------------------------------
#include "std_msgs/msg/char.hpp"

// ----------------------- NAMESPACES CONSIDERED ------------------------------
using namespace std::chrono_literals;

// ------------------------ ASCII CLASS IMPLEMENTATION ------------------------

/**
 * Struct oriented to create a intra process communication by just using one
 * node that implements a publisher and subscriber in itself.
 */
struct IncrementerASCII : public rclcpp::Node
{
    /**
     * Constructor that initialize the node with the given name, configures
     * intra process communication (both by using the parents constructor),
     * then create a publisher and makes use of it with a weak_ptr, then
     * creates a subscriber where a lambda callback is linked where it
     * receives a message and then it publishes an increment of it.
     * 
     * NOTE: You need to create a first publication in order to make
     * available the communication.
     */
    IncrementerASCII(const std::string & name, const std::string & in, 
        const std::string & out)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        // Instance a publisher that considers an output channel.
        pub = this->create_publisher<std_msgs::msg::Char>(out, 10);

        // Use weak ptr to create instance of the publisher
        std::weak_ptr<std::remove_pointer<decltype(pub.get())>::type> 
            captured_pub = pub;

        // Instance a subscription for char message, link an in channel
        // and implement a lambda callback for the communication interaction.
        sub = this->create_subscription<std_msgs::msg::Char>
        (
            in,
            10,
            [captured_pub](std_msgs::msg::Char::UniquePtr msg)
            {
                // Using the weak_ptr (passed to the lambda), get a shared ptr
                // to access the pub and verify
                auto pub_ptr = captured_pub.lock();
                if(!pub_ptr)
                {
                    return;
                }

                // Display last msg received
                printf("Letter received: %c | Memory address: 0x%" PRIXPTR "\n",
                    msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
                
                // Add a delay (wait time)
                if(!rclcpp::sleep_for(2s))
                {
                    return;
                }

                // Consider ASCII numeric code and increment it
                int ascii = (int) msg->data;
                if( ++ascii > 126 )
                {
                    ascii = 33;
                }

                // Update content of msg and display before publication
                msg->data = (char)(ascii);
                printf("Incrementing ASCII Value: %c | Memory address: 0x%"
                    PRIXPTR "\n", msg->data, 
                    reinterpret_cast<std::uintptr_t>(msg.get()));

                // Publish message
                pub_ptr->publish(std::move(msg));
            }
        );
    }

    // Define publisher and subscriber
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr pub;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr sub;
};

// ---------------------- MAIN IMPLEMENTATION ---------------------------------
int main(int argc, char* argv[])
{
    // Set buffering mode in this case for stdout stream (console), NULL (
    // buffer managed by itself), _IONBF (no buffering, output direct to
    // console) and BUFSIZ (ignored due to _IONBF)
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize node and create executor
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor st_exec;

    // Instance 2 intances of the same node for the future communication 
    auto ascii1 = std::make_shared<IncrementerASCII>("first_ascii_node", 
        "channel1", "channel2");
    auto ascii2 = std::make_shared<IncrementerASCII>("second_ascii_node",
        "channel2", "channel1");
    rclcpp::sleep_for(2s);

    // Create new message by using unique_ptr (to prevent copy) and init it
    std::unique_ptr<std_msgs::msg::Char> msg(new std_msgs::msg::Char());
    msg->data = '!';

    // Create a publication to start the cyclic communication for the 2
    // nodes we define
    printf("First (and only manual publication) letter: %c | Memory Address: 0x%" 
        PRIXPTR "\n", msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
    ascii1->pub->publish(std::move(msg));

    // Link nodes to the executor and spin
    st_exec.add_node(ascii1);
    st_exec.add_node(ascii2);
    st_exec.spin();

    // When everything is done, cloase the program
    rclcpp::shutdown();
    return 0;
}