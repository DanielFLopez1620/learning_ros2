# -------------------- CMake description for package --------------------------
# CMake basic info of project
cmake_minimum_required(VERSION 3.8)
project(m02_ros2_with_cpp)

# Compiler specificacion for C++
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------- Dependencies and packages related ---------------------
# Find other packages that are dependencies, must be congruent with the ones in
# the package.xml
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#  Test specifications
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------- Adding executables --------------------------------
# Int64 Publisher
add_executable(pub_int64 src/int64_pub.cpp)
ament_target_dependencies(pub_int64 rclcpp std_msgs)
# Int64 Subscriber
add_executable(sub_int64 src/int64_sub.cpp)
ament_target_dependencies(sub_int64 rclcpp std_msgs)

# Adding Two Nums Server
add_executable(add_nums_srv src/add_two_nums_srv.cpp)
ament_target_dependencies(add_two_nums_srv rclcpp std_msgs)

# Adding Two Nums Client
add_executable(add_nums_cli src/add_two_nums_cli.cpp)
ament_target_dependencies(add_two_nums_cli rclcpp std_msgs)

# Adding targets to the libraries
install(TARGETS
  pub_int64
  sub_int64
  DESTINATION lib/${PROJECT_NAME})

# Specify ament
ament_package()
