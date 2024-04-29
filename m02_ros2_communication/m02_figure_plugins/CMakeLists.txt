# ------------------- BASE PACKAGE DEFINITIONS ---------------------------------
cmake_minimum_required(VERSION 3.8)
project(m02_figure_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Requiere dependencies 
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(m02_base_figure REQUIRED)
find_package(pluginlib REQUIRED)

# Line required to link the plugins description and also consider the base class
pluginlib_export_plugin_description_file(m02_base_figure plugins.xml)


# ------------------- EXECUTABLES AND LIBRARIES SET UP -------------------------

# Specify source code of the library for the plugin
add_library(figure_plugins src/m02_figure_plugins.cpp)

# Specify compilation with C99 and C++17
target_compile_features(figure_plugins PUBLIC c_std_99 cxx_std_17)

# Link the info with the header definition and also add dependencies
target_include_directories(figure_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  figure_plugins
  "m02_base_figure"
  "pluginlib"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(figure_plugins PRIVATE "M02_FIGURE_PLUGINS_BUILDING_LIBRARY")

# Consider the files in the include directory 
install(
  DIRECTORY include/
  DESTINATION include
)

# Add executables to library destination, but also export library
install(
  TARGETS figure_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ------------------------ TEST CONFIGURATION ---------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#  ---------------------- FINAL AMENT INCLUDES ---------------------------------

# Required to consider header definition, which has the base figure class
ament_export_include_directories(
  include
)

# Export library for the plugin
ament_export_libraries(
  figure_plugins
)

# Export project targets for usage in other packages
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
