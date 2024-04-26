#include <pluginlib/class_loader.hpp>
#include <m02_base_figure/base_figure.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<m02_base_figure::BaseFigure> figure_loader("m02_base_figure", "m02_base_figure::BaseFigure");

  try
  {
    std::shared_ptr<m02_base_figure::BaseFigure> circle = figure_loader.createSharedInstance("m02_figure_plugins::Circle");
    circle->initialize(2.0);

    printf("Circle area: %.2f\n", circle->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}
