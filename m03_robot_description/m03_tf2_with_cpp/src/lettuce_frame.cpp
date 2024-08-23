// ---------------------- STANDARD HEADERS REQUIRED ---------------------------
#include <chrono>      // Time management with different precisions
#include <functional>  // For function usage and hash related
#include <memory>      // Dynamic memory management

// ----------------------- ROS2 RELATED HEADERS -------------------------------
#include "rclcpp/rclcpp.hpp"                 // ROS2 Client Library for C++
#include "tf2_ros/transform_broadcaster.h"   // Tf broadcaster

// ------------------------ ROS2 MESSAGES -------------------------------------
#include "geometry_msgs/msg/transform_stamped.hpp"  // Tf with time stamp

// ------------------------ NAMESPACE CONSIDERATIONS --------------------------
using namespace std::chrono_literals;  // For user-defined chrono literals


// --------------------- FRAME BROADCASTER IMPLEMENTATION --------------------

/**
 * Class that publishes a static frame from the turtle, you can take it as the 
 * fishing rot with carrot example, that always stays at the same position from
 * the turtle, like in Minecraft with pigs.
 */
class LettuceFrameBroadcaster : public rclcpp::Node
{
public:
    /**
     * User defined constructor that initialize the node under the name 
     * "fixed_lettuce_tf2_broadcaster", declare a broadcaster and a timer
     * linked with a callback.
     */
    LettuceFrameBroadcaster() 
        : Node("fixed_lettuce_tf2_broadcaster")
    {
        tf_broad_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            250ms, std::bind(&LettuceFrameBroadcaster::broad_callback, this));
    }

private:
    /**
     * Callback that takes place when a timer specifies it, to broadcast a 
     * transform that stays fixed relative to the turtle1
     */
    void broad_callback()
    {
        // Declare stamped transform
        geometry_msgs::msg::TransformStamped t_stamp;

        // Add header information: Time, parent and child
        t_stamp.header.stamp = this->get_clock()->now();
        t_stamp.header.frame_id = "turtle1";
        t_stamp.child_frame_id = "lettuce";
        
        // Constant traslational values
        t_stamp.transform.translation.x = 0.0;
        t_stamp.transform.translation.y = 2.0;
        t_stamp.transform.translation.z = 0.0;

        // Constant rotational values (Quaternion)
        t_stamp.transform.rotation.x = 0.0;
        t_stamp.transform.rotation.y = 0.0;
        t_stamp.transform.rotation.z = 0.0;
        t_stamp.transform.rotation.w = 1.0;

        // Publish transform
        tf_broad_->sendTransform(t_stamp);
    }

    // Declare timer and transform broadcaster
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_;
};

// -------------------- MAIN IMPLEMENTATION -----------------------------------

int main(int argc, char * argv[])
{
    // Initialize ROS2 Client Library for C++
    rclcpp::init(argc, argv);

    // Spin node creation
    rclcpp::spin(std::make_shared<LettuceFrameBroadcaster>());
    
    // When spin is interrupted, close and shutdown
    rclcpp::shutdown();
    return 0;
}