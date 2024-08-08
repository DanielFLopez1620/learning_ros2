#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class LettuceFrameBroadcaster : public rclcpp::Node
{
public:
    LettuceFrameBroadcaster() 
        : Node("fixed_lettuce_tf2_broadcaster")
    {
        tf_broad_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            250ms, std::bind(&LettuceFrameBroadcaster::broad_callback, this));
    }

private:
    void broad_callback()
    {
        geometry_msgs::msg::TransformStamped t_stamp;

        t_stamp.header.stamp = this->get_clock()->now();
        t_stamp.header.frame_id = "turtle1";
        t_stamp.child_frame_id = "lettuce";
        
        t_stamp.transform.translation.x = 0.0;
        t_stamp.transform.translation.y = 2.0;
        t_stamp.transform.translation.z = 0.0;

        t_stamp.transform.rotation.x = 0.0;
        t_stamp.transform.rotation.y = 0.0;
        t_stamp.transform.rotation.z = 0.0;
        t_stamp.transform.rotation.w = 1.0;

        tf_broad_->sendTransform(t_stamp);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LettuceFrameBroadcaster>());
    rclcpp::shutdown();
    return 0;
}