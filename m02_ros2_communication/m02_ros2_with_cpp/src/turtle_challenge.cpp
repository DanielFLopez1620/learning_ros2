// ---------------------- CPP Standard Headers --------------------------------
#include <cmath>
#include <chrono>

// ---------------------- ROS2 Headers ----------------------------------------
#include <rclcpp/rclcpp.hpp>

// ---------------------- Messages and Services needed -----------------------
#include <turtlesim/msg/pose.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/srv/teleport_relative.hpp>

// ------------------- Namespaces used ---------------------------------------
using namespace std::chrono_literals;

// -------------------- Playground Class Implementation -----------------------
class PlaygroundTurtle : public rclcpp::Node
{
public:
    // User defined constructor
    PlaygroundTurtle() : Node("playground_turtle_cpp") {}

    /**
     * Call service to clear turtlesim background
     * 
     * @return true if response is received without errors.
    */
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

    /**
     * Call service to create new turtle
     * 
     * @param x Set X position of the turtle in the turtlesim map. 
     * @param y Set Y position of the turtle in the turtlesim map.
     * @param theta Set orientation of the turtle (radians)
     * @param name Name of the turtle create
     * 
     * @return true if response is received without errors.
    */
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

    /**
     * Call service to delete a turtle
     * 
     * @param name Name of the turtle to delete
     * 
     * @return true if response is received without errors.
    */
    bool kill_turtle(std::string& name)
    {
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;
        while(!this->kill_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                "Waiting for service in turtlesim: Kill");
        }
        auto result = kill_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        return false;
    }

    /**
     * Call service to teleport the turtle (according origin of turtlesim)
     * 
     * @param x Set X position to tp turtle.
     * @param y Set Y position to tp turtle.
     * @param theta Set orientation of turtle (radians)
     * @param name Turtle to teleport
     * 
     * @return true if response is received without errors.
    */
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
                "Waiting for service in turtlesim: Teleport Absolute");
        }
        auto result = tp_abs_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        return false;
    }

    /**
     * Call service to teleport the turtle (veloicity of given turtle)
     * 
     * @param linear Set linear velocity of the turlte
     * @param angular Set angular velocity of the turtle
     * @param name Name of the turtle teleport.
     * 
     * @return true if response is received without errors.
    */
    bool teleport_rel_turtle(float linear, float angular, std::string& name)
    {
        tp_rel_client_ = this->create_client<turtlesim::srv::TeleportRelative>
            ( "/" + name + "/teleport_relative");
        auto request = 
            std::make_shared<turtlesim::srv::TeleportRelative::Request>();
        request->linear = linear;
        request->angular = angular;
        while(!this->tp_rel_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                "Waiting for service in turtlesim: Teleport Relative");
        }
        auto result = tp_rel_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        return false;
    }

    /**
     * Call service to teleport the turtle (according origin of turtlesim)
     * 
     * @param r 0-255 intensity of red color
     * @param g 0-255 intensity of green color
     * @param b 0-255 intensity of blue color
     * @param witdh Width of the pen trance
     * @param name Name of the turtle to modify the pen configuration
     * 
     * @return true if response is received without errors.
    */
    bool set_pen_turtle(unsigned int r,unsigned int g, unsigned int b, 
        unsigned int width, std::string& name)
    {
        pen_client_ = this->create_client<turtlesim::srv::SetPen>
            ( "/" + name + "/set_pen");
        auto request = 
            std::make_shared<turtlesim::srv::SetPen::Request>();
        request->r = r;
        request->g = g;
        request->b = b;
        request->width = width;
        request->off = 0;
        while(!this->pen_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                "Waiting for service in turtlesim: SetPen");
        }
        auto result = pen_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        return false;
    }

private:
    // Client definitions for turtlesim
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_ = 
        this->create_client<std_srvs::srv::Empty>("/clear");
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_ = 
        this->create_client<turtlesim::srv::Spawn>("/spawn");
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_ = 
        this->create_client<turtlesim::srv::Kill>("/kill");
    
    // Client definition for turtles
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr tp_abs_client_;
    rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedPtr tp_rel_client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;

    // Subscriptions
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
};

int main(int argc, char* argv[])
{
    // Initialize ROS Client Library for C++
    rclcpp::init(argc, argv);

    // Define name and instance a playground object
    std::string name1 {"dan_turtle"};
    auto ptr_play = std::make_shared<PlaygroundTurtle>();

    // Play with the turtle
    ptr_play->clear_board();
    ptr_play->spawn_turtle(3, 3, 0, name1);
    ptr_play->teleport_abs_turtle(7, 3, 3*M_PI_2, name1);
    ptr_play->set_pen_turtle(255, 0, 0, 2, name1);
    ptr_play->teleport_abs_turtle(7, 7, -M_PI, name1);
    ptr_play->set_pen_turtle(0, 255, 0, 2, name1);
    ptr_play->teleport_abs_turtle(3, 7, M_PI_2, name1);
    ptr_play->set_pen_turtle(0, 0, 255, 2, name1);
    ptr_play->teleport_abs_turtle(3, 3, M_PI_2, name1);
    ptr_play->teleport_rel_turtle(1, M_PI_2, name1);
    ptr_play->kill_turtle(name1);
    
    // Close and shutdown
    rclcpp::shutdown();
    return 0;
}