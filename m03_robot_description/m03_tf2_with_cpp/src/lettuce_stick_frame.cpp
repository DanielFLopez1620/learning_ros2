#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class LettuceStickBroadcaster : public rclcpp::Node
{
public:
    LettuceStickBroadcaster()
        : Node("lettuce_stick_broadcaster")
    {
        tf_broad_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&LettuceStickBroadcaster::broad_callback, this));
    }
private:
    void broad_callback()
    {
        rclcpp::Time now = this->get_clock()->now();
        double x = now.seconds() * M_PI;

        geometry_msgs::msg::TransformStamped t_stamp;
        t_stamp.header.stamp = now();
        t_stamp.header.frame_id = "turtle1";
        t_stamp.child_frame_id = "lettuce1";
        t_stamp.transform.translation.x = 10 * sin(x);
        t_stamp.transform.translation.y = 10 * cos(x);
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
    rclcpp::spin(std::make_shared<LettuceStickBroadcaster>());
    rclcpp::shutdown();
    return 0;
}