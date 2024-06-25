#ifndef M02_ROS2_WITH_CPP__INT64_PUB_COMP_
#define M02_ROS2_WITH_CPP__INT64_PUB_COMP_

#include "rclcpp/rclcpp.hpp"
#include "m02_ros2_with_cpp/visibility_control.h"

#include "std_msgs/msg/int64.hpp"

namespace example_comp
{
    class IntPub : public rclcpp::Node
    {
    public:
        M02_ROS2_WITH_CPP_PUBLIC
        explicit IntPub(const rclcpp::NodeOptions & options);

    protected:
        void on_timer();
        
    private:
        int element_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

} // namespace example_comp


#endif  // M02_ROS2_WITH_CPP__INT64_PUB_COMP_