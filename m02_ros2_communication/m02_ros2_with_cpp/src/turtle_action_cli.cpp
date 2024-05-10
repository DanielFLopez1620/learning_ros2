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
        using namespace std::placeholders;
        this->timer_->cancel();

        if(!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server isn't available... Exiting");
            rclcpp::shutdown();
        }
        int num_moves = 5;
        auto goal_msg = RegularMove::Goal();
        goal_msg.num_moves = num_moves;

        RCLCPP_INFO(this->get_logger(), "Sending goal: Num of moves of %d", num_moves);

        auto send_goal_options = rclcpp_action::Client<RegularMove>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&TurtleActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&TurtleActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&TurtleActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
private:
    rclcpp_action::Client<RegularMove>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(const GoalHandlerRegularMove::SharedPtr & goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal wasn't accepted by server...");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted... Waiting result...");
        }
    }

    void feedback_callback(
        GoalHandlerRegularMove::SharedPtr,
        const std::shared_ptr<const RegularMove::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Current move is:" << feedback->current_move << "\n";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandlerRegularMove::WrappedResult & result)
    {
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was completed...");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted...");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled...");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unkwown code received...");
                return;
        }
        std::stringstream ss;
        ss << "Turtle completed the action...\n Moves achieved:" 
            << result.result->moves;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(TurtleActionClient)