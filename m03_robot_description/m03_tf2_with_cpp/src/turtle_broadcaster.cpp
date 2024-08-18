// ---------------------- REQUIRED STANDARD HEADERS ---------------------------
#include <functional>  // For function object and standard hash function
#include <memory>      // Dynamic memory management
#include <sstream>     // String stream
#include <string>      // Collection of capabilites for string usage

// ----------------------- ROS2 REQUIRED HEADERS -----------------------------
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// ----------------------- ROS2 MSGS DEPENDENCIES ----------------------------
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "turtlesim/msg/pose.hpp"

// ----------------------- NAMESPACE CONSIDERATION --------------------------- 
using namespace std::placeholders; // Argument placement

// ------------------ BROADCASTER CLASS IMPLEMENTATION ------------------------
/**
 * Classs that broadcast the tf of a turtle (dynamic) in the turtlesim world,
 * so it allows to obtain info of its position.
 */
class TurtlePoseBroad : public rclcpp::Node
{
public:
    /**
     * User defined constructor that initalize a node with the name 
     * "turtle_tf2_frame_publisher". Also, it declares a turtle param, a tf
     * broadcaster and subscription to a poser.
     */
    TurtlePoseBroad()
        : Node("turtle_tf2_frame_publisher")
    {
        // Parameter declaration
        turtlename_ = this->declare_parameter<std::string>(
            "turtlename", "turtle");
        
        // Definition of tf broadcaster by using unique_ptr
        tf_broad_ = 
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Obtain the corresponding topic pose by considering turtle name param
        std::ostringstream stream;
        stream << "/" << turtlename_.c_str() << "/pose";
        std::string topic_name = stream.str();

        // Create subscription and link handler of the pose to broadcast tf
        subs_ = this->create_subscription<turtlesim::msg::Pose>(
            topic_name, 10, 
            std::bind(&TurtlePoseBroad::handle_turtle_pose, this, _1));
    }

private:
    /**
     * Function that receives the msg pose of the turtle and broadcast it with
     * tfs.
     * 
     * @param msg Pose msg received of the given turtle
     */
    void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
    {
        // Declare transfor stamped message
        geometry_msgs::msg::TransformStamped t_stamp;

        // Add stamp (time, child frame and parent frame)
        t_stamp.header.stamp = this->get_clock()->now();
        t_stamp.header.frame_id = "world";
        t_stamp.child_frame_id = turtlename_.c_str();

        // Add traslational info of the turtle (considering just 2D components)
        t_stamp.transform.translation.x = msg->x;
        t_stamp.transform.translation.y = msg->y;
        t_stamp.transform.translation.z = 0.0;

        // Traslate RPY to quaternion to obtain the given position
        // Herez z is just considered as a 2D rotation axis
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);

        // Add the rotational info of the turtle
        t_stamp.transform.rotation.x = q.x();
        t_stamp.transform.rotation.y = q.y();
        t_stamp.transform.rotation.z = q.z();
        t_stamp.transform.rotation.w = q.w();

        // Broadcast transform
        tf_broad_->sendTransform(t_stamp);
    }

    // Declare subscription via shared_ptr
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subs_;

    // Declare tf broadcaster via unique_ptr
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broad_;

    // Add string for turtlename
    std::string turtlename_;
};

// ------------------------- MAIN IMPLMENTATION -------------------------------
int main(int argc, char * argv[])
{
    // Initialize ROS2 Client Library for C++
    rclcpp::init(argc, argv);

    // Spin node by using shared pointer
    rclcpp::spin(std::make_shared<TurtlePoseBroad>());
    
    // Close and shutdown when the spin ends
    rclcpp::shutdown();
    return 0;
}