#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <turtlesim/msg/pose.hpp>

#include <std_srvs/srv/empty.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/srv/teleport_relative.hpp>

using namespace std::chrono_literals;

class PlaygroundTurtle : public rclcpp::Node
{
public:
    PlaygroundTurtle() : Node("playground_turtle_cpp") {}

    bool clear_board()
    {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        while(!this->clear_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                "Waiting for service in turtlesim: Clear");
        }
        auto result = clear_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        return false;
    }

    bool spawn_turtle(float x, float y, float theta, std::string& name)
    {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        request->name = name;
        while(!this->spawn_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                "Waiting for service in turtlesim: Spawn");
        }
        auto result = spawn_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        return false;
    }

    bool kill_turtle(std::string& name)
    {
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;
        while(!this->kill_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                "Waiting for service in turtlesim: Spawn");
        }
        auto result = kill_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        return false;
    }

    bool teleport_abs_turtle(float x, float y, float theta, std::string& name)
    {
        tp_abs_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>
            ( "/" + name + "/teleport_absolute");
        auto request = 
            std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        while(!this->tp_abs_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                "Waiting for service in turtlesim: Spawn");
        }
        auto result = tp_abs_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        return false;
    }

private:
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_ = 
        this->create_client<std_srvs::srv::Empty>("/clear");
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_ = 
        this->create_client<turtlesim::srv::Spawn>("/spawn");
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_ = 
        this->create_client<turtlesim::srv::Kill>("/kill");
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr tp_abs_client_;
    rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedPtr tp_rel_client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
};