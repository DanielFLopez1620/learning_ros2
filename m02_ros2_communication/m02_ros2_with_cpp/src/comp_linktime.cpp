// ------------------------- STANDARD HEADER REQUIRED -------------------------
#include <memory>  // For dynamic memory management
#include <string>  // String methods and work with chain of characters
#include <vector>  // For dynamic containers management
#include <utility> // Collection of general utility funcions

// ------------------------- RCLCPP REQUIRED HEADERS -------------------------
#include "class_loader/class_loader.hpp"      // Mechanism for loading dyn lib
#include "rclcpp/rclcpp.hpp"                  // For C++ with ROS2
#include "rclcpp_components/node_factory.hpp" // Abstract creation of nodes

// Logger name
#define COMP_LINKTIME_LOGGER_NAME "comp_linktime"

// -------------------------- MAIN IMPLEMENTATION ----------------------------
int main(int argc, char* argv[])
{
    // For forcing flushing of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize ROS2 client library for C++
    rclcpp::init(argc, argv);
    
    // Create a logger with the name defined previously
    rclcpp::Logger logger = rclcpp::get_logger(COMP_LINKTIME_LOGGER_NAME);
    
    // Definition of executor for managing node's execution
    rclcpp::executors::SingleThreadedExecutor st_exec;

    // Instance the default initialization options for a node
    rclcpp::NodeOptions options;

    // Vector declared for holding instances of the class loader (mecahism
    // oriented to dynaically load shared libraries at runtime and preventing
    // the statical link with them) and the node wrapper (for storing the
    // nodes loaded)
    std::vector<std::unique_ptr<class_loader::ClassLoader>> loaders;
    std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;

    // Vector that should contain the paths to the library of interest
    std::vector<std::string> libraries = { "", };

    // For each oriented to iterate throuhg available libraries.
    for(auto lib : libraries)
    {
        RCLCPP_INFO(logger, "Loading library %s", lib.c_str());
        // Create a loader instance for the library detected
        auto loader = std::make_unique<class_loader::ClassLoader>(lib);
        
        // Obtain the available classes that can be implemented with NodeFactory
        auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
        
        // For each class detected...
        for (auto ecl : classes)
        {
            RCLCPP_INFO(logger, "Creating instance of: %s", ecl.c_str());
            // Instance node that can be used with NodeFactory

            auto node_fact = loader->createInstance<rclcpp_components::NodeFactory>(ecl);
            // Add it to a temporal wrapper while passing the options for init
            auto wrapper = node_fact->create_node_instance(options);
            auto node = wrapper.get_node_base_interface();
            node_wrappers.push_back(wrapper);
            
            // Add it to the executor
            st_exec.add_node(node);
        }

        // Save the loader in the vector
        loaders.push_back(std::move(loader));
    }

    // Spin for using the callbacks of the node
    st_exec.spin();

    // When an interrupt is called, it removes the wrapped nodes
    for (auto wrapper : node_wrappers)
    {
        st_exec.remove_node(wrapper.get_node_base_interface());
    }

    // Finally, shutdown and return.
    rclcpp::shutdown();
    return 0;
}