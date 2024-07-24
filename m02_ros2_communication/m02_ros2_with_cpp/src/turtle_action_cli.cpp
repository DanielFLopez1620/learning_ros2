// ---------------------------- STANDARD HEADERS -----------------------------

#include <functional>       // For general functions of objects and hash func
#include <future>          // To access ot results of asynchronous operations
#include <memory>         // Dynamic memory management
#include <string>         // Numeric constants and math functions
#include <sstream>        // For output operations
#include <chrono>         // For time usage iwth different precisions
#include <bits/stdc++.h>  // Part numeric library, oriented to distributions

// -------------------------- ROS2  ACTIONS ----------------------------------

#include "m02_ros2_with_cpp/action/regular_move.hpp" // Action to implement

// -------------------------- ROS 2 RCLCPP REQUIRED --------------------------

#include "rclcpp/rclcpp.hpp"                          // For C++ with ROS2
#include "rclcpp_action/rclcpp_action.hpp"            // For using actions
#include "rclcpp_components/register_node_macro.hpp"  // Node register for lib

// ------------------------- TURTLE ACTION CLIENT IMPL -----------------------

/**
 * Class oriented to create an action client for implementing geometric moves
 * (regular polygons) while playing with the turtle1 in the turtlesim node.
*/
class TurtleActionClient : public rclcpp::Node
{
public:
    // Type alias for the action type
    using RegularMove = m02_ros2_with_cpp::action::RegularMove;

    // Type alias for the goal handler
    using GoalHandlerRegularMove = rclcpp_action::ClientGoalHandle<RegularMove>;

    /**
     * Explicit constructor that initialize the node 'turtle_action_cli' and pass
     * the node options, then create an action client and a timer.
     * 
     * @param options Node options (related with the call of library 
     *                implemented).
    */
    explicit TurtleActionClient(const rclcpp::NodeOptions & options)
    : Node("turtle_action_cli", options)
    {
        // Define action client by considering action type alias
        this->client_ptr_ = rclcpp_action::create_client<RegularMove>(
            this,
            "turtle_mov");

        // Define timer linked to a callback to send the goal
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TurtleActionClient::send_goal, this));

        RCLCPP_INFO(this->get_logger(), "Action client initialized...");
    }
    
    /**
     * Function to send a random goal, so the turtle1 will draw a polygon
     * according it. It will also link the goal, feedback and action
     * callbacks for this process.
    */
    void send_goal()
    {
        // Placeholders definition for args hablde according position.
        using namespace std::placeholders;

        // Cancel timer after first exection
        this->timer_->cancel();

        // Check if action server is active, otherwise, shut down the node
        if(!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Action server isn't available... Exiting");
            rclcpp::shutdown();
        }

        // Generate random number for the size
        auto seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine gen(seed);
        std::uniform_int_distribution<int> distro(4, 20);
        auto num_moves = distro(gen);

        // Instance and define goal.
        auto goal_msg = RegularMove::Goal();
        goal_msg.num_moves = num_moves;
        RCLCPP_INFO(this->get_logger(), 
            "Sending goal: Num of moves of %d", num_moves);

        // Instance the client seding goal options
        auto send_goal_options = 
            rclcpp_action::Client<RegularMove>::SendGoalOptions();

        // Create options for respond, feedback and result with callbacks 
        send_goal_options.goal_response_callback =
            std::bind(&TurtleActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&TurtleActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&TurtleActionClient::result_callback, this, _1);

        // Asynchronous send of the goal
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    // Instance action client
    rclcpp_action::Client<RegularMove>::SharedPtr client_ptr_;

    // Instance timer
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * Callback to check for response of the goal, will display if the goal was
     * accepted or not.
     * 
     * @param goal_handler Handler goal to obtain status of action goal.
    */
    void goal_response_callback(
        const GoalHandlerRegularMove::SharedPtr & goal_handle)
    {
        // Check return of goal handle
        if(!goal_handle)
        {
            // Error in case of not accepted
            RCLCPP_ERROR(this->get_logger(), 
                "Goal wasn't accepted by server...");
        }
        else
        {
            // Log if accepted
            RCLCPP_INFO(this->get_logger(), 
                "Goal accepted... Waiting result...");
        }
    }

    /**
     * Callback to interpret and receive feedback, as it will display the 
     * current move of the turtle.
     * 
     * @param feedback Pointer to feedback action structure
    */
    void feedback_callback(
        GoalHandlerRegularMove::SharedPtr,
        const std::shared_ptr<const RegularMove::Feedback> feedback)
    {
        // Create chaing to append feedback received
        std::stringstream ss;
        ss << "Current move is:" << feedback->current_move << "\n";

        // Log info of current feedback
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    /**
     * Callback for getting the result, to display if the action was
     * completed, aborted, canceled or it isn't known. If completed
     * will display the result (but you should be able to see the 
     * polygon completed in the turtlesim window)
     * 
     * @param result Pointer to result action
    */
    void result_callback(const GoalHandlerRegularMove::WrappedResult & result)
    {
        // Check code result
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was completed...");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted...");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled...");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unkwown code received...");
                return;
        }

        // If the result was completed, it will display it
        std::stringstream ss;
        ss << "Turtle completed the action...\n Moves achieved:" 
            << result.result->moves;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        RCLCPP_INFO(this->get_logger(), "Action client finishing...");
        // Close node when achieved
        rclcpp::shutdown();
    }
};

// Macro oriented to launch of classes as nodes
RCLCPP_COMPONENTS_REGISTER_NODE(TurtleActionClient)