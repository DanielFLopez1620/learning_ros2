#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

#define COMP_LINKTIME_LOGGER_NAME "comp_linktime"

int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    
    rclcpp::Logger logger = rclcpp::get_logger(COMP_LINKTIME_LOGGER_NAME);
    rclcpp::executors::SingleThreadedExecutor st_exec;
    rclcpp::NodeOptions options;

    std::vector<std::unique_ptr<class_loader::ClassLoader>> loaders;
    std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;

    std::vector<std::string> libraries = { "", };

    for(auto lib : libraries)
    {

    }

    st_exec.spin()

    rclcpp::shutdown();

    return 0;
}