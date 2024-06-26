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
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

#  Test specifications
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------- Interface generation ----------------------------------

set(msg_files
  "msg/RarePoint.msg"
)

set(srv_files
  "srv/Answer.srv")

set(act_files
  "action/RegularMove.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  # You can manually add your srv and msg file, like this:
  # "msg/RarePoint.msg"
  # "srv/Answer.srv"

  # But it is better to use the sets.
  ${msg_files}
  ${srv_files}
  ${act_files}
  DEPENDENCIES std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)


# ------------------------- Adding executables --------------------------------
# Int64 Publisher
add_executable(pub_int64 src/int64_pub.cpp)
ament_target_dependencies(pub_int64 rclcpp std_msgs)

# Int64 Subscriber
add_executable(sub_int64 src/int64_sub.cpp)
ament_target_dependencies(sub_int64 rclcpp std_msgs)

# Adding Two Nums Server
add_executable(add_nums_srv src/add_two_nums_srv.cpp)
ament_target_dependencies(add_nums_srv rclcpp example_interfaces)

# Adding Two Nums Client
add_executable(add_nums_cli src/add_two_nums_cli.cpp)
ament_target_dependencies(add_nums_cli rclcpp example_interfaces)

# RarePoint Publisher
add_executable(rand_xy_pub src/rand_xy_pub.cpp)
ament_target_dependencies(rand_xy_pub rclcpp)

# Exam Server (for answers)
add_executable(exam_srv src/exam_srv.cpp)
ament_target_dependencies(exam_srv rclcpp)

# Saying Hi Publisher (with params)
add_executable(saying_hi src/saying_hi.cpp)
ament_target_dependencies(saying_hi rclcpp)

# Simple turtle mov
add_executable(simple_turtle_mov src/simple_turtle_mov.cpp)
ament_target_dependencies(simple_turtle_mov rclcpp geometry_msgs)

add_executable(turtle_challenge src/turtle_challenge.cpp)
ament_target_dependencies(turtle_challenge rclcpp turtlesim std_srvs)

# Usage of own definitions in the same package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(rand_xy_pub "${cpp_typesupport_target}")
target_link_libraries(exam_srv "${cpp_typesupport_target}")

# Adding turtle_action_srvs
add_library(turtle_action_server SHARED
  src/turtle_action_srv.cpp)

target_include_directories(turtle_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(turtle_action_server
  PRIVATE "M02_ROS2_WITH_CPP_BUILDING_DLL") 

ament_target_dependencies(turtle_action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs")

target_link_libraries(turtle_action_server "${cpp_typesupport_target}")

rclcpp_components_register_node(turtle_action_server PLUGIN "TurtleActionServer" EXECUTABLE turtle_action_srv)

# Adding turtle_action_cli
add_library(turtle_action_client SHARED
  src/turtle_action_cli.cpp)

target_include_directories(turtle_action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(turtle_action_client
  PRIVATE "M02_ROS2_WITH_CPP_BUILDING_DLL") 

ament_target_dependencies(turtle_action_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

target_link_libraries(turtle_action_client "${cpp_typesupport_target}")

rclcpp_components_register_node(turtle_action_client PLUGIN "TurtleActionClient" EXECUTABLE turtle_action_cli)

# Adding intra process communication for randonm float
add_executable(intra_com_2_nodes src/intra_com_2_nodes.cpp)
target_link_libraries(intra_com_2_nodes rclcpp::rclcpp ${std_msgs_TARGETS})

# Adding intra process communication for cyclic incrementer
add_executable(intra_com_1_node src/intra_com_1_node.cpp)
target_link_libraries(intra_com_1_node rclcpp::rclcpp ${std_msgs_TARGETS})

# Adding int64_pub component
add_library(int64_pub_component SHARED
  src/int64_pub_comp.cpp)
target_compile_definitions(int64_pub_component
  PRIVATE "MO2_ROS2_WITH_CPP_BUILDING_DLL")
ament_target_dependencies(int64_pub_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(int64_pub_component "example_comp::IntPub")
set(node_plugins "${node_plugins}example_comp::IntPub;$<TARGET_FILE:int64_pub_component>}\n")

set(libs
  int64_pub_component)

# Adding targets to the libraries
install(TARGETS
  pub_int64
  sub_int64
  add_nums_cli
  add_nums_srv
  rand_xy_pub
  exam_srv
  saying_hi
  simple_turtle_mov
  turtle_challenge
  intra_com_2_nodes
  intra_com_1_node
  DESTINATION lib/${PROJECT_NAME})

# Adding targets for actions
install(TARGETS
  turtle_action_server
  turtle_action_client
  int64_pub_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# Launch files
install(DIRECTORY
  launch config include
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------- Ament final configs------------------------

# Specify ament
ament_package()
