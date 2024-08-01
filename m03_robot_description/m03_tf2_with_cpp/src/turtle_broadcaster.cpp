#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

using namespace std::placeholders;

class TurtlePoseBroad : public rclcpp::Node
{
public:
    TurtlePoseBroad()
        : Node("turtle_tf2_frame_publisher")
    {
        turtlename_ = this->declare_parameter<std::string>("turtlename", "follower");

        tf_broad_ = 
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::ostringstream stream;
        stream << "/" << turtlename_.c_str() << "/pose";
        std::string topic_name = stream.str();

        subs_ = this->create_subscription<turtlesim::msg::Pose>(
            topic_name, 10, 
            std::bind(&TurtlePoseBroad::handle_turtle_pose, this, _1));
    }

private:
    void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
    {
        geometry_msgs::msg::TransformStamped t_stamp;

        t_stamp.header.stamp = this->get_clock()->now();
        t_stamp.header.frame_id = "world";
        t_stamp.child_frame_id = turtlename_.c_str();

        t_stamp.transform.translation.x = msg->x;
        t_stamp.transform.translation.y = msg->y;
        t_stamp.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t_stamp.transform.rotation.x = q.x();
        t_stamp.transform.rotation.x = q.y();
        t_stamp.transform.rotation.x = q.z();
        t_stamp.transform.rotation.x = q.w();

        tf_broad_->sendTransform(t_stamp);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subs_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broad_;
    std::string turtlename_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlePoseBroad>());
    rclcpp::shutdown();

    return 0;
}