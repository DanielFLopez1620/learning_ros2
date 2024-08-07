#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class TurtleListener : public rclcpp::Node
{
public:
    TurtleListener()
        : Node("turtle_tf2_frame_listener"),
          turtle_srv_is_ready_(false),
          turtle_spawned_(false)
    {
        target_frame_ = this->declare_parameter<std::string>("target_frame",
            "turtle1");
        
        tf_buf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listen_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "explorer/cmd_vel", 1);

        timer_ = this->create_wall_timer(1s, 
            std::bind(&TurtleListener::on_timer, this));
        
    }
private:
    void on_timer()
    {
        std::string fromFrameRel = target_frame_.c_str();
        std::string toFrameRel = "follower";

        if(turtle_srv_is_ready_)
        {
            if(turtle_spawned_)
            {
                geometry_msgs::msg::TransformStamped t_stamp;

                try
                {
                    t_stamp = tf_buf_->lookupTransform(
                        toFrameRel, fromFrameRel, tf2::TimePointZero);
                }
                catch(const tf2::TransformException & ex)
                {
                    RCLCPP_INFO(
                        this->get_logger(), "Could find transform between"
                        "%s to %s: %s", toFrameRel.c_str() , 
                        fromFrameRel.c_str(), ex.what()
                    );
                    return;
                }

                geometry_msgs::msg::Twist msg;

                static const double scaleRotationRate = 1.0;
                msg.angular.z = scaleRotationRate * atan2(
                    t_stamp.transform.translation.y,
                    t_stamp.transform.translation.x);

                static const double scaleForwardSpeed = 0.5;
                msg.linear.x = scaleForwardSpeed * sqrt(
                    pow(t_stamp.transform.translation.x, 2) +
                    pow(t_stamp.transform.translation.y, 2));

                cmd_vel_pub_->publish(msg);
            }
            else
            {
                if (spawner_->service_is_ready())
                {
                    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                    request->x = 4.0;
                    request->y = 2.0;
                    request->theta = 0.0;
                    request->name = "follower";

                    using ServiceResponseFuture = 
                        rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
                    
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
                    auto result = spawner_->async_send_request(request, resp_rec_callback);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Service is not ready");
                }
            }
        }
    }
    bool turtle_srv_is_ready_;
    bool turtle_spawned_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listen_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buf_;
    std::string target_frame_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleListener>());
    rclcpp::shutdown();
    return 0;
}