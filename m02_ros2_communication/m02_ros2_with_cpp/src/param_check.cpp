// ---------------------------- REQUIRED STANDARD HEADERS ---------------------
#include <memory>

// ---------------------------- RCLCPP REQUIRED DEPENDENCIES ------------------
#include "rclcpp/rclcpp.hpp"

// ---------------------------- PARAMETER STORAG IMPLEMENTATION ---------------

/**
 * Class oriented to check the updates of two paramaters: "desired_num" and
 * "your_name", while providing logs when the updates happens. 
 */
class ParameterStorage : public rclcpp::Node
{
public:
    /**
     * User-defined constructor to declare the "desired_num" param, and 
     * implement to lambda callback for the "desired_num" and "your_name"
     * params while also linking the callbacks to the param_handler
     * functions.
     */
    ParameterStorage() : Node("parameter_storage")
    {
        // Base declaration and definition of a parameter
        this->declare_parameter("desired_num", 1.6);

        // Instance a parameter event handler
        param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // Lambda oriented to display logs about the updates of the double
        // value of the desired number.
        auto update_callback_1 = [this](const rclcpp::Parameter & p)
        {
            RCLCPP_INFO(this->get_logger(), 
                "Notification: Param [%s] with type [%s] was updated to: %f",
                p.get_name().c_str(), p.get_type_name().c_str(),
                p.as_double());
        };

        // Lambda oriented to display logs about the updates of the string
        // value of the your_name param
        auto update_callback_2 = [this](const rclcpp::Parameter & p)
        {
            RCLCPP_INFO(this->get_logger(), 
                "Update: Param [%s] with type [%s] was updated to: %s",
                p.get_name().c_str(), p.get_type_name().c_str(),
                p.as_string().c_str());
        };

        // Link handler for the "desired_num"
        param_callback_ = param_handler_->add_parameter_callback(
            "desired_num", update_callback_1);

        // Link handler for a paramater in other node (saying_your name), as
        // the name of the parameter is "your_name"
        auto saying_name_node = std::string("saying_your_name");
        auto saying_name_param = std::string("your_name");
        saying_callback_ = param_handler_->add_parameter_callback(
            saying_name_param, update_callback_2, saying_name_node);

    } // ParameterStorage()

private:

    // Private definition for Param Event Handler and Callback Handler
    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> saying_callback_;
};

int main(int argc, char* argv[])
{
    // Initialize ROS2 Client Library for C++
    rclcpp::init(argc, argv);

    // Sping node
    rclcpp::spin(std::make_shared<ParameterStorage>());
    
    // Close and shutdown after sping interruption (Ctrl+C or related)
    rclcpp::shutdown();
    return 0;
}