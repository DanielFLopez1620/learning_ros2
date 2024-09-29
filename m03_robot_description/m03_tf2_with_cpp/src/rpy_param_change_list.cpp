#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

using namespace std::placeholders;

class RPYParamListener : public rclcpp::Node
{
    public:
        RPYParamListener() : Node("rpy_listener")
        {
            parameter_event_sub_ = 
                this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
                    "/parameter_events", 10,
                    std::bind(&RPYParamListener::on_event, this, _1));
        }

        void on_event(
            const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
        {
            for (const auto & changed_parameter : event->changed_parameters)
            {
                if(changed_parameter.name == "roll" 
                    || changed_parameter.name == "pitch"
                    || changed_parameter.name == "yaw")
                {
                    RCLCPP_INFO(this->get_logger(), "Parameter changed: %s",
                        changed_parameter.name.c_str());
                }
            }
        }
    private: 
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr 
            parameter_event_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RPYParamListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}