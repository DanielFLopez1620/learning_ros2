#include <memory>
#include <string>
#include <vector>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

#define COMP_DLOPEN_LOGGER_NAME "comp_dlopen"

int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    if(argc < 2)
    {
        fprintf(stderr, "Need one arg passed to load library");
        return 1;
    }

    rclcpp::init(argc, argv);
    
    rclcpp::Logger logger = rclcpp::get_logger(COMP_DLOPEN_LOGGER_NAME);
    
    rclcpp::executors::SingleThreadedExecutor st_exec;

    rclcpp::NodeOptions options;

    std::vector<class_loader::ClassLoader *> loaders;
    std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;

    std::vector<std::string> libs;
    for(int i = 1; i < argc; ++i)
    {
        libs.push_back(argv[i]);
    }

    for (auto lib : libs)
    {
        RCLCPP_INFO(logger, "Loading passed library: %s", lib.c_str());
        auto loader = new class_loader::ClassLoader(lib);
        auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
        for (auto ecl : classes)
        {
            RCLCPP_INFO(logger, "Instanciating class: %s", ecl.c_str());
            auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(ecl);
            auto wrapper = node_factory->create_node_instance(options);
            auto node = wrapper.get_node_base_interface();
            node_wrappers.push_back(wrapper);
            st_exec.add_node(node);
        }
        loaders.push_back(loader);
    }
    
    st_exec.spin();

    for (auto wrapper : node_wrappers) 
    {
        st_exec.remove_node(wrapper.get_node_base_interface());
    }
    node_wrappers.clear();

    rclcpp::shutdown();

    return 0;

}