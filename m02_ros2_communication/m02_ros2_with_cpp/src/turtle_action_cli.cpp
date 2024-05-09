#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "m02_ros2_with_cpp/action/regular_move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class TurtleActionClient : public rclcpp::Node
{
public:
    using RegularMove = m02_ros2_with_cpp::action::RegularMove;
    using GoalHandlerRegularMove = rclcpp_action::ServerGoalHandle<RegularMove>;

    explicit TurtleActionClient(const rclcpp::NodeOptions & options)
    : Node("turtle_action_cli", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<RegularMove>(
            this,
            "turtle_mov");
    }

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&TurtleActionClient::send_goal, this));

    void send_goal()
    {
        
    }


};