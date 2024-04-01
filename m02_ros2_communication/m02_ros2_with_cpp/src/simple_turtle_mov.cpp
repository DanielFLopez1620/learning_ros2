#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/twist.hpp"

class SimpleTurtleMove : public rclcpp::Node
{
public:
    SimpleTurtleMove() : Node("simple_turtle_mov")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
    }

    void single_publish()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 2.0;
        msg.angular.z = 3.1416 / 2;
        publisher_->publish(msg);
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto ptr = std::make_shared<SimpleTurtleMove>();
    ptr->single_publish();

    rclcpp::spin(ptr);
    
    rclcpp::shutdown();
    return 0;
}