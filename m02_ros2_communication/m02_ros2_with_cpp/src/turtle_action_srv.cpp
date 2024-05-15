// -------------------------------- STANDARD HEADERS --------------------------

#include <functional>     // For general functions of objects and hash funcs.
#include <memory>         // Dynamic memory management
#include <thread>         // For thread support (multithreading programming)
#include <bits/stdc++.h>  // Part numeric library, oriented to distributions
#include <cmath>          // Numeric constants and math functions
#include <chrono>         // For time usage iwth different precisions

// ------------------------------- ROS2 MSGS AND ACTIONS ----------------------

#include "m02_ros2_with_cpp/action/regular_move.hpp"   // Action to implement
#include "geometry_msgs/msg/twist.hpp"                 // For cmd_vel topics.

// ------------------------------ ROS2 RCLCPP REQUIRED ------------------------

#include "rclcpp/rclcpp.hpp"                           // For C++ with ROS2
#include "rclcpp_action/rclcpp_action.hpp"             // For using actions
#include "rclcpp_components/register_node_macro.hpp"   // Node register for lib
#include "m02_ros2_with_cpp/visibility_control.h"      // Visibility

// ----------------------------- TURTLE ACTION SERVER IMPL --------------------

/**
 * Class oriented to create an action server for implementing geometric moves
 * (regular polygons) while playing with the turtle1 in the turlesim node.
*/
class TurtleActionServer : public rclcpp::Node
{
public:
    // Type aliase for action type
    using RegularMove = m02_ros2_with_cpp::action::RegularMove;

    // Type alise for the goal handler
    using GoalHandlerRegularMove = 
        rclcpp_action::ServerGoalHandle<RegularMove>;

    M02_ROS2_WITH_CPP_PUBLIC
    /**
     * Explict constructor that initialize the node 'turtle_action_srv' and
     * pass the node options. Then it links the handles for the states of the
     * action (goal, cancel and accept).
    */
    explicit TurtleActionServer(const rclcpp::NodeOptions & options = 
        rclcpp::NodeOptions())
    : Node("turtle_action_srv", options)
    {
        // Placeholders will interact with the std::bind call to represent the
        // position for arguments in a function call. In this case for the
        // handles.
        using namespace std::placeholders;

        // Define an action server by using the alias action defined previously,
        // and link the corresponding handlers.T
        this->action_server_ = rclcpp_action::create_server<RegularMove>(
            this,
            "turtle_mov",
            std::bind(&TurtleActionServer::handle_goal, this, _1, _2),
            std::bind(&TurtleActionServer::handle_cancel, this, _1),
            std::bind(&TurtleActionServer::handle_accepted, this, _1));
    }
private:
    // Instance an action server by considering the alias action defined
    rclcpp_action::Server<RegularMove>::SharedPtr action_server_;

    /**
     * In case that a goal proposal is recieved, it will check and acccept it
     * 
     * @param uuid Pointer action goal id
     * @param goal Shared pointer to the goal to consider
     * 
     * @return ACCEPT_AND_EXECUTE, to follo next steps in action.
    */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const RegularMove::Goal> goal)
    {
        // Display info of the goal
        RCLCPP_INFO(this->get_logger(), 
            "Objective was set... Num moves: %d", goal->num_moves);

        // Use goal id (Suppress unused variable warning)
        (void)uuid;

        // Return status of goal response
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * In case of cancelation, accept the canceleation order.
     * 
     * @param goal_handle Pointer to goal handler for the regular move.
     * 
     * @return Status accepted for cancellation.
    */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlerRegularMove> goal_handle)
    {
        // Display info of cancelation
        RCLCPP_INFO(this->get_logger(), "Cancelation order received...");

        // Use goal handle (Suppress unused variable warning)
        (void)goal_handle;

        // Return status of accepted cancelation (request)
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * In case of accepted goal, use multithreading and execute goal.
     * 
     * @param goal_handle Pointer to goal handler for the regular move.
    */
    void handle_accepted(
        const std::shared_ptr<GoalHandlerRegularMove> goal_handle)
    {
        // For function params location.
        using namespace std::placeholders;

        // Use a thread to create a multithreading execution of the goal
        std::thread{std::bind(&TurtleActionServer::execute, this, _1),
            goal_handle}.detach();
    }

    /**
     * Execute the turtle regular move (polygon) by considering the goal
     * proposed and initializing a random velocity for the turtle moves. It 
     * will also update the feedback and inform when achieved.
     * 
     * @param goal_handle Pointer to handler of regular move, including goal.
    */
    void execute(const std::shared_ptr<GoalHandlerRegularMove> goal_handle)
    {
        // Log message for execution
        RCLCPP_INFO(this->get_logger(), "Executing goal...");

        // Rate considered for delays.
        rclcpp::Rate loop_rate(0.5);

        // Declare and instance goal and feedback for action
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<RegularMove::Feedback>();

        // Generate random number for lineal velocity
        auto seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine gen(seed);
        std::uniform_real_distribution<float> distro(1.0, 5.0);
        auto dist = distro(gen);
        
        // Create a result (will be updated when goal is completed)
        auto result = std::make_shared<RegularMove::Result>();

        // Create publisher for /turtle1/cmd_vel
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Instance geometry message, and initalize rotation and feedback
        auto msg = geometry_msgs::msg::Twist();
        auto rot = 2* M_PI / goal->num_moves;
        feedback->current_move = 0;

        // Loop that considers the number of moves received in the goal
        for(auto i = 0; i < goal->num_moves; ++i)
        {
            // Check if cancelation is received
            if(goal_handle->is_canceling())
            {
                // Updates result with feedback (at current point)
                result->moves = feedback->current_move;

                // Launch cancelation and log process.
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                return;
            }

            // Make rotational move (according goal to achieve regultar polygon)
            msg.linear.x = 0;
            msg.angular.z = rot;
            cmd_vel_publisher_->publish(msg);
            loop_rate.sleep();

            // Make linear move (will be the same to keep regular polygon)
            msg.linear.x = dist;
            msg.angular.z = 0;
            cmd_vel_publisher_->publish(msg);
            loop_rate.sleep();

            // Update feedback, then publish and log it
            (feedback->current_move)++;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publishing feedback...");  
        }
    
        // If node is still working...
        if(rclcpp::ok())
        {
            // Update result (and it must be corresponding with the goal)
            result->moves = feedback->current_move;

            // Send success, publish and log result.
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal completed");
        }
    }
};

// Macro oriented to launch of classes as nodes
RCLCPP_COMPONENTS_REGISTER_NODE(TurtleActionServer)