#include <functional>
#include <memory>
#include <thread>

#include "m02_ros2_with_cpp/action/regular_move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "m02_ros2_with_cpp/visibility_control.h"

class TurtleActionServer : public rclcpp::Node
{
public:
    M02_ROS2_WITH_CPP_PUBLIC
    explicit TurtleActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("turtle_action_server", options)
    {
        using namespace std::placeholders;

        this->action_server = rclcpp_action::create_server<RegularMove>(
            this,
            'turtle_mov',
            std::bind(&TurtleActionServer::handle_goal, this, _1, _2),
            std::bind(&TurtleActionServer::handle_cancel, this, _1),
            std::bind(&TurtleActionServer::handle_accepted, this, _1),
        )
    }
private:
    rclcpp_action::Server<RegularMove>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Objective was set... Num moves: %d", goal->num_moves);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleRegularMov> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Cancelation order received...")
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
};


int main(int argc, char* argv[])
{
    return 0;
}