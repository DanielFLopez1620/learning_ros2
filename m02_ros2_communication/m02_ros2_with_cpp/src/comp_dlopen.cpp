// ----------------------- REQUIRED LIBRARIES ---------------------------------
#include <memory>  // For dynamic memory mangament
#include <string>  // Operations and methods for strings
#include <vector>  // Related with dynamic contianers

// ----------------------- RCLCPP REQUIRED DEPENDENCIES -----------------------
#include "rclcpp/rclcpp.hpp"                   // For C++ with ROS2
#include "rclcpp_components/node_factory.hpp"  // Abstract creation of nodes
#include "class_loader/class_loader.hpp"       // Mechanism for dynamic libs

// Logger name
#define COMP_DLOPEN_LOGGER_NAME "comp_dlopen"

/*
NOTES FOR USAGE WITH ROS2:
    ros2 run m02_ros2_with_cpp comp_dlopen `ros2 pkg prefix m02_ros2_with_cpp
    `/lib/libint64_pub_component.so `ros2 pkg prefix m02_ros2_with_cpp`
    /lib/libint64_sub_component.so
*/

// ------------------------------ MAIN IMPLEMENTATION -------------------------
int main(int argc, char* argv[])
{
    // For forcing flusing of stdout and ensure immediate output
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Checking for at leat to argument (additional to the ros2 run <exec>...)
    if(argc < 2)
    {
        fprintf(stderr, "Need one arg passed to load library");
        return 1;
    }

    // Initialize ROS2 client library for C++
    rclcpp::init(argc, argv);
    
    // Create logger from previously defined name
    rclcpp::Logger logger = rclcpp::get_logger(COMP_DLOPEN_LOGGER_NAME);
    
    // Instance executor for future nodes links
    rclcpp::executors::SingleThreadedExecutor st_exec;

    // Instance default options for nodes
    rclcpp::NodeOptions options;

    // Defines a holder for the loaders and node wrappers
    std::vector<class_loader::ClassLoader *> loaders;
    std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;

    // Consider the arguments passed (in form of ROS2 pkg prefix command)
    // and stores it in a library vector names
    std::vector<std::string> libs;
    for(int i = 1; i < argc; ++i)
    {
        libs.push_back(argv[i]);
    }

    // For each library considered....
    for (auto lib : libs)
    {
        // Log the library that is being loaded
        RCLCPP_INFO(logger, "Loading passed library: %s", lib.c_str());
        
        // Creates a new class loader for the library
        auto loader = new class_loader::ClassLoader(lib);

        // Get the available clases that implement Node Factory
        auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
        
        // For each class
        for (auto ecl : classes)
        {
            // Log the installation of the class
            RCLCPP_INFO(logger, "Instanciating class: %s", ecl.c_str());
            
            // Instance a node factory 
            auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(ecl);
            
            // Create a wrapper for the node factory just created
            auto wrapper = node_factory->create_node_instance(options);
            auto node = wrapper.get_node_base_interface();
            node_wrappers.push_back(wrapper);
            
            // Add the instance node to the executor
            st_exec.add_node(node);
        }

        // Save the loader considered
        loaders.push_back(loader);
    }
    
    // spin for using the callbacks of the nodes
    st_exec.spin();

    // For each wrapper created, remove everything when done
    for (auto wrapper : node_wrappers) 
    {
        st_exec.remove_node(wrapper.get_node_base_interface());
    }

    // Finally clear the wrapper and shutdown
    node_wrappers.clear();
    rclcpp::shutdown();

    return 0;

}