#include <memory>
#include <string>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp/parameter.hpp"

class RPYToQuaternionPublisher : public rclcpp::Node
{
public:
    RPYToQuaternionPublisher()
        : Node("rpy_to_quaternion_publisher")
    {
        // Declare and initialize parameters
        this->declare_parameter("roll", 0.0);
        this->declare_parameter("pitch", 0.0);
        this->declare_parameter("yaw", 0.0);
        this->declare_parameter("frame_id", "world");
        this->declare_parameter("child_frame_id", "rpy_frame");

        // Initialize TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Set a timer to periodically broadcast the transform
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&RPYToQuaternionPublisher::broadcast_transform, this));
    }

private:
    void broadcast_transform()
    {
        // Get the current RPY values from parameters
        double roll, pitch, yaw;
        this->get_parameter("roll", roll);
        this->get_parameter("pitch", pitch);
        this->get_parameter("yaw", yaw);

        // Convert RPY to quaternion
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        // Fill the TransformStamped message
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = this->get_parameter("frame_id").as_string();
        transform_stamped.child_frame_id = this->get_parameter("child_frame_id").as_string();
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create the RPYToQuaternionPublisher node
    auto node = std::make_shared<RPYToQuaternionPublisher>();

    // Spin the node to process callbacks
    rclcpp::spin(node);

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    return 0;
}
