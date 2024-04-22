cmake_minimum_required(VERSION 3.8)
project(m02_plugins_figure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(m02_base_figure REQUIRED)

add_library(m02_plugins_figure src/plugins_figure.cpp)
target_compile_features(m02_plugins_figure PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(m02_plugins_figure PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  m02_plugins_figure
  "m02_base_figure"
  "pluginlib"
)

pluginlib_export_plugin_description_file(base_figure plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(m02_plugins_figure PRIVATE "M02_plugins_figure_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS m02_plugins_figure
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  m02_plugins_figure
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
