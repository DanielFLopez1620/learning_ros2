#include <cstdio>
#include <pluginlib/class_loader.hpp>
#include <m02_base_figure/base_figure.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<base_figure::BaseFigure> figure_loader("m02_base_figure", "base_figure::BaseFigure");

  try
  {
      std::shared_ptr<base_figure::BaseFigure> circle = figure_loader.createSharedInstance("plugins_figure::Circle");
      circle->initialize(2.5);
      printf("Given Circle Area: %.2f\n ", circle->area());
  }
  catch(const pluginlib::PluginlibException& ex)
  {
      printf("The plugin failed to load, error message: %s\n", ex.what());
  }
  return 0;
}
