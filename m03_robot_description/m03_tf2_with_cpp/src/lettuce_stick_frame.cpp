// --------------------  STANDARD HEADERS REQUIRED -----------------------------
#include <chrono>      // Time mangement with different precisions
#include <functional>  // For function usage and hash related
#include <memory>      // Dynamic memory management
#include <cmath>       // C constants from math and operations 

// ------------------------ ROS2 RELATED HEADERS ------------------------------
#include "rclcpp/rclcpp.hpp"                // ROS2 Client Library for C++
#include "tf2_ros/transform_broadcaster.h"  // TF broadcaster

// ------------------------ ROS2 MESSAGES -------------------------------------
#include "geometry_msgs/msg/transform_stamped.hpp" // Tf with time stamp

// ------------------------ NAMESPACE CONSIDERATIONS --------------------------
using namespace std::chrono_literals;  // For user-defined chrono literals

// ----------------- DYNAMIC FRAME BROADCASTER IMPLEMENTATION -----------------

/**
 * Class that publishes a dynamic frame from the turtle, you can understand it
 * as it was a firshing rot with a carrot with a lot of wind, a very long
 * stick and a very long rope, this is made by using sine and cosine functions
 */
class LettuceStickBroadcaster : public rclcpp::Node
{
public:
    /**
     * User defined constructor that initialize the node under the name 
     * "lettue_stick_broadcaster", declare a broadcaster and a timer linked
     * with a callback to publish the tf.
     */
    LettuceStickBroadcaster()
        : Node("lettuce_stick_broadcaster")
    {
        // Declare a transform broadcater
        tf_broad_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Timer that will be linked with the publish and broadcast fo the tf
        timer_ = this->create_wall_timer(
            100ms, std::bind(&LettuceStickBroadcaster::broad_callback, this));
    }
private:
    /**
     * Callback that takes place when a timer request it, to broadcast a
     * tranform that is dynamically related to the turtle and circle 
     * identitites
     */
    void broad_callback()
    {
        // Get current time and process it with pi
        rclcpp::Time now = this->get_clock()->now();
        double x = now.seconds() * M_PI;

        // Declare stamped transform
        geometry_msgs::msg::TransformStamped t_stamp;

        // Add header information: Time parent and child
        t_stamp.header.stamp = now;
        t_stamp.header.frame_id = "turtle1";
        t_stamp.child_frame_id = "lettuce1";

        // X and Y traslational values related with sine and cosine of time
        t_stamp.transform.translation.x = 10 * sin(x);
        t_stamp.transform.translation.y = 10 * cos(x);
        t_stamp.transform.translation.z = 0.0;

        // Rotation remains const
        t_stamp.transform.rotation.x = 0.0;
        t_stamp.transform.rotation.y = 0.0;
        t_stamp.transform.rotation.z = 0.0;
        t_stamp.transform.rotation.w = 1.0;

        // Publish transform
        tf_broad_->sendTransform(t_stamp);
    }

    // Declare timer and transfor broadcaster
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS2 Cleint Library for C++
    rclcpp::init(argc, argv);

    // Spin node creation
    rclcpp::spin(std::make_shared<LettuceStickBroadcaster>());

    // When spin is interrupted, close and shutdown
    rclcpp::shutdown();
    return 0;
}