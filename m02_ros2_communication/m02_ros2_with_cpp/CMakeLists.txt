cmake_minimum_required(VERSION 3.8)
project(m02_ros2_with_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(pub_int64 src/int64_pub.cpp)
ament_target_dependencies(pub_int64 rclcpp std_msgs)

add_executable(sub_int64 src/int64_sub.cpp)
ament_target_dependencies(sub_int64 rclcpp std_msgs)

install(TARGETS
  pub_int64
  sub_int64
  DESTINATION lib/${PROJECT_NAME})



ament_package()
