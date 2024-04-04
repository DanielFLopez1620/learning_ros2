// --------------------- ROS2 Client Libraries --------------------------------
#include "rclcpp/rclcpp.hpp" 

// --------------------- Message required -------------------------------------
#include "geometry_msgs/msg/twist.hpp"

// ------------------- SimpleTurtleMove implementation -----------------------
class SimpleTurtleMove : public rclcpp::Node
{
public:
    /**
     * Constructor that initialize node and set a publisher of twist
     * messages for /cmd_vel topics.
    */
    SimpleTurtleMove() : Node("simple_turtle_mov")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
    }

    /**
     * Publsih a single twist message to a turtle
    */
    void single_publish()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 2.0;
        msg.angular.z = 3.1416 / 2;
        publisher_->publish(msg);
    }
private:
    // Instance publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS Client Library for Python
    rclcpp::init(argc, argv);

    // Instance class and publish the single message
    auto ptr = std::make_shared<SimpleTurtleMove>();
    ptr->single_publish();

    // Spin to make sure message is published
    rclcpp::spin(ptr);
    
    // Close and shutdown
    rclcpp::shutdown();
    return 0;
}