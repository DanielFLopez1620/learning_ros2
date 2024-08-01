cmake_minimum_required(VERSION 3.8)
project(m03_tf2_with_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(static_broadcaster src/static_broadcaster.cpp)
ament_target_dependencies(
  static_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(turtle_broadcaster src/turtle_broadcaster.cpp)
ament_target_dependencies(
  turtle_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)

install(TARGETS
  static_broadcaster
  turtle_broadcaster
  DESTINATION lib/${PROJECT_NAME}  
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
