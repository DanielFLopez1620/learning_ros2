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
        RCLCPP_INFO(logger, "Loading library %s", lib.c_str());
        auto loader = std::make_unique<class_loader::ClassLoader>(lib);
        auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
        for (auto ecl : classes)
        {
            RCLCPP_INFO(logger, "Creating instance of: %s", ecl.c_str());
            auto node_fact = loader->createInstance<rclcpp_components::NodeFactory>(ecl);
            auto wrapper = node_fact->create_node_instance(options);
            auto node = wrapper.get_node_base_interface();
            node_wrappers.push_back(wrapper);
            st_exec.add_node(node);
        }
        loaders.push_back(std::move(loader));
    }

    st_exec.spin();

    for (auto wrapper : node_wrappers)
    {
        st_exec.remove_node(wrapper.get_node_base_interface());
    }

    rclcpp::shutdown();

    return 0;
}