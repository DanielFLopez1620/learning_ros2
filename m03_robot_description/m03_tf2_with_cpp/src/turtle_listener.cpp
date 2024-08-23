// ------------------------------ REQUIRED STANDARD HEADERS --------------------
#include <chrono>     // Time management with different precisions
#include <functional> // For functions and hash related
#include <memory>     // Dynamic memory management
#include <string>     // String utilities

// ------------------------------ ROS2 RELATED HEADERS ------------------------
#include "rclcpp/rclcpp.hpp"             // ROS2 Client Library for C++
#include "tf2/exceptions.h"              // Transform exceptions
#include "tf2_ros/transform_listener.h"  // Transform listener
#include "tf2_ros/buffer.h"              // Transform buffer

// ----------------------------- ROS2 MSGS DEPENDENCIES -----------------------
#include "geometry_msgs/msg/transform_stamped.hpp"   // Tf with time stamped
#include "geometry_msgs/msg/twist.hpp"               // Velocity command

// ----------------------------- ROS2 SRVS DEPENDENCIES -----------------------
#include "turtlesim/srv/spawn.hpp"   // Service to create new turtles

// ----------------------------- NAMESPACES CONSIDERATIONS --------------------
using namespace std::chrono_literals;  // For time user defined literals

// ----------------------- TF LISTERNER CLASS IMPLEMENTATION -----------------

/**
 * Creates nodes that will listen to the tranforms in order to implement a tf
 * follower with turtles.
 */
class TurtleListener : public rclcpp::Node
{
public:
    /**
     * User defined constructor that initialize the node with the name 
     * turtle_tf2_frame_lister, declares a target frame, a transform buffer, 
     * a turtle spawner server client and a cmd_vel publisher to create the
     * transform listerner linked to a timer callback.
     */
    TurtleListener()
        : Node("turtle_tf2_frame_listener"),
          turtle_srv_is_ready_(false),
          turtle_spawned_(false)
    {
        // Consider declared parameter for the name
        target_frame_ = this->declare_parameter<std::string>("target_frame",
            "turtle1");
        
        // Buffer that has current time information
        tf_buf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        // Instance transform listener that considers a pointer to the buffer
        tf_listen_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

        // Instance a spawner server client
        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");

        // Instance a cmd_vel publisher to create follower of tfs
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "follower/cmd_vel", 1);

        // Implement timer with it sproper callback
        timer_ = this->create_wall_timer(1s, 
            std::bind(&TurtleListener::on_timer, this));
    }
private:
    /**
     * Callback for the timer that will lookup for the transform between the 
     * turtles, then interpret the distnace and make the other turtle follow 
     * the first one.
     * 
     * If the turtle that follows the first one doesn't extis, it will spawn a
     * new one.
     * 
     */
    void on_timer()
    {
        // Obtain and assign names of the transform of interest
        std::string fromFrameRel = target_frame_.c_str();
        std::string toFrameRel = "follower";

        // Check if the spawn service is ready
        if(turtle_srv_is_ready_)
        {
            // Check if the follower turtle spawned
            if(turtle_spawned_)
            {
                // Declare stamped transform
                geometry_msgs::msg::TransformStamped t_stamp;

                try
                {
                    // Look up for transform as soon as it is available
                    t_stamp = tf_buf_->lookupTransform(
                        toFrameRel, fromFrameRel, tf2::TimePointZero);
                }
                catch(const tf2::TransformException & ex)
                {
                    // Display message in case of error for tfs loop up
                    RCLCPP_INFO(
                        this->get_logger(), "Could find transform between"
                        "%s to %s: %s", toFrameRel.c_str() , 
                        fromFrameRel.c_str(), ex.what()
                    );
                    return;
                }

                // Declare twist message for velocity commands
                geometry_msgs::msg::Twist msg;

                // Assign rotation based on the x and y distance between turtle
                static const double scaleRotationRate = 1.0;
                msg.angular.z = scaleRotationRate * atan2(
                    t_stamp.transform.translation.y,
                    t_stamp.transform.translation.x);

                // Assign linear velocity based on Pitagoras Theorem scaled
                static const double scaleForwardSpeed = 0.5;
                msg.linear.x = scaleForwardSpeed * sqrt(
                    pow(t_stamp.transform.translation.x, 2) +
                    pow(t_stamp.transform.translation.y, 2));

                // Publish velocity command
                cmd_vel_pub_->publish(msg);
            }
            else
            {
                // Log to show that turtle was spawned
                RCLCPP_INFO(this->get_logger(), "Succesfully spawned turtle");
                turtle_spawned_ = true;
            }
        }
        else
        {
            // If the service is ready....
            if (spawner_->service_is_ready())
            {
                // Make the request to spawn a turtle in the givne position
                auto request = 
                    std::make_shared<turtlesim::srv::Spawn::Request>();
                request->x = 4.0;
                request->y = 2.0;
                request->theta = 0.0;
                request->name = "follower";

                // Create future response
                using ServiceResponseFuture = 
                    rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
                
                // Lambda for checking that the turtle was created
                auto resp_rec_callback = [this](ServiceResponseFuture future)
                {
                    auto result = future.get();
                    if(strcmp(result->name.c_str(), "follower") == 0)
                    {
                        turtle_srv_is_ready_ = true;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Spawn didn't work");
                    }
                };

                // Send request to spawn turtle with verification
                auto result = spawner_->async_send_request(request, 
                    resp_rec_callback);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Service is not ready");
            }
        }
    }
    // Declare flags of services
    bool turtle_srv_is_ready_;
    bool turtle_spawned_;

    // Declare string for parameter
    std::string target_frame_;

    // Declare ros related private attributes for service, timer, publisher, tf
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listen_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buf_;
};

// ------------------------- MAIN IMPLEMENTATION ------------------------------

int main(int argc, char * argv[])
{
    // Initialize ROS2 Client Library for C++
    rclcpp::init(argc, argv);

    // Spin node by using shared pointer
    rclcpp::spin(std::make_shared<TurtleListener>());
    
    // When spin ends, shutdown and close
    rclcpp::shutdown();
    return 0;
}