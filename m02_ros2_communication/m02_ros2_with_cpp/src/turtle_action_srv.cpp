#include <functional>
#include <memory>
#include <thread>
#include <bits/stdc++.h>
#include <cmath>

#include "m02_ros2_with_cpp/action/regular_move.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "m02_ros2_with_cpp/visibility_control.h"

class TurtleActionServer : public rclcpp::Node
{
public:
    using RegularMove = m02_ros2_with_cpp::action::RegularMove;
    using GoalHandlerRegularMove = rclcpp_action::ServerGoalHandle<RegularMove>;

    M02_ROS2_WITH_CPP_PUBLIC
    explicit TurtleActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("turtle_action_srv", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<RegularMove>(
            this,
            "turtle_mov",
            std::bind(&TurtleActionServer::handle_goal, this, _1, _2),
            std::bind(&TurtleActionServer::handle_cancel, this, _1),
            std::bind(&TurtleActionServer::handle_accepted, this, _1));
    }
private:
    rclcpp_action::Server<RegularMove>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const RegularMove::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Objective was set... Num moves: %d", goal->num_moves);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlerRegularMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Cancelation order received...");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlerRegularMove> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&TurtleActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlerRegularMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<RegularMove::Feedback>();

        std::default_random_engine gen;
        std::uniform_real_distribution<float> distro(1.0, 5.0);
        auto dist = distro(gen);
        
        // Pending to design turtlebot moves
        auto result = std::make_shared<RegularMove::Result>();

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        auto msg = geometry_msgs::msg::Twist();
        auto rot = M_PI / goal->num_moves;
        feedback->current_move = 0;

        for(auto i = 0; i < goal->num_moves; ++i)
        {
            if(goal_handle->is_canceling())
            {
                result->moves = feedback->current_move;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                return;
            }

            msg.linear.x = 0;
            msg.angular.z = rot;
            cmd_vel_publisher_->publish(msg);

            loop_rate.sleep();

            msg.linear.x = dist;
            msg.angular.z = 0;
            cmd_vel_publisher_->publish(msg);

            loop_rate.sleep();

            (feedback->current_move)++;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publishing feedback...");  
        }
    
        if(rclcpp::ok())
        {
            result->moves = feedback->current_move;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal completed");
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(TurtleActionServer)