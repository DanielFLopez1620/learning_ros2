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
        : Node("quaternion_publisher")
    {
        // Declare and initialize parameters
        this->declare_parameter("qx", 0.0);
        this->declare_parameter("qy", 0.0);
        this->declare_parameter("qz", 0.0);
        this->declare_parameter("qw", 0.0);
        this->declare_parameter("frame_id", "world");
        this->declare_parameter("child_frame_id", "quaternion_frame");

        // Initialize TransformBroadcaster
        tf_broad_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Set a timer to periodically broadcast the transform
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&RPYToQuaternionPublisher::broadcast_transform, this));
    }

private:
    void broadcast_transform()
    {
        // Get the current RPY values from parameters
        double qx, qy, qz, qw;
        this->get_parameter("qx", qx);
        this->get_parameter("qy", qy);
        this->get_parameter("qz", qz);
        this->get_parameter("qw", qw);

        // Convert RPY to quaternion
        tf2::Quaternion q;

        // Fill the TransformStamped message
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = 
            this->get_parameter("frame_id").as_string();
        transform_stamped.child_frame_id = 
            this->get_parameter("child_frame_id").as_string();
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = qx;
        transform_stamped.transform.rotation.y = qy;
        transform_stamped.transform.rotation.z = qz;
        transform_stamped.transform.rotation.w = qw;

        // Display passed quaternion, if you want to learn more about 
        // quaternions, visualization and math, you can go to:
        // https://eater.net/quaternions
        RCLCPP_INFO(this->get_logger(), "Quaternion received is: %f, %f, %f, %f",
            qx, qy, qz, qw);


        // Mathematical conversion from quaternion to ray, pitch, yaw.
        // Based on: Automatic Addison Tutorial on Conversions.
        // https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        double r = 
            atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
        double p = 2.0 * (qw * qy - qz * qx);
        if(p > 1.0)
            p = 1.0;
        else if (p < -1.0)
            p = -1.0;
        double y = 
            atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        // Display obtained RPY
        RCLCPP_INFO(this->get_logger(), "Equivalent RPY is: %f, %f, %f", r, p, y);
        RCLCPP_INFO(this->get_logger(), "----------------------------------");


        // Broadcast the transform
        tf_broad_->sendTransform(transform_stamped);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_;
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
