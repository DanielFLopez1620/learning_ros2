// ------------------------ C++ STANDARD LIBRARIES -----------------------------
#include <iostream>

// ---------------------- PLUGINLIBS HEADERS -----------------------------------
#include <pluginlib/class_loader.hpp>        // Loader required

// ---------------------- PLUGINS DEFINITIONS ----------------------------------
#include <m02_base_figure/base_figure.hpp>   // Base class inclussion

// --------------------- MAIN IMPLEMENTATION -----------------------------------
int main(int argc, char** argv)
{
    // Load base class present in header file
    pluginlib::ClassLoader<m02_base_figure::BaseFigure> 
        figure_loader("m02_base_figure", "m02_base_figure::BaseFigure");

    // Use plugin under test environment
    try
    {
        // Access to the plugin info to instance a circle object.
        std::shared_ptr<m02_base_figure::BaseFigure> circle = 
            figure_loader.createSharedInstance("m02_figure_plugins::Circle");
        
        // Set main dimension (diameter in this case)
        circle->initialize(2.0);

        // Use area method and display the result
        std::cout << "Circle area: " <<  circle->area()); << std::endl;
    }
    catch(pluginlib::PluginlibException& ex)
    {
        // Show message in case of result not being displayed.
        std::cout << "The plugin failed to load for some reason. Error\n: "
                  << ex.what() << std::endl;
    }

    return 0;
}
