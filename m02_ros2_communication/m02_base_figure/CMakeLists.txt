# ------------------- BASE PACKAGE DEFINITIONS ---------------------------------
cmake_minimum_required(VERSION 3.8)
project(m02_base_figure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Requiere dependencies 
find_package(ament_cmake REQUIRED) # Package build
find_package(pluginlib REQUIRED)   # Usage of plugins

# ------------------- EXECUTABLES AND LIBRARIES SET UP -------------------------

# Consider executable of the area calculator
add_executable(area_calculator src/area_calculator.cpp)

# Link the info with the header definitions
target_include_directories(area_calculator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Specify compilation with C99 and C++17
target_compile_features(area_calculator PUBLIC c_std_99 cxx_std_17)

# Link dependencies with pluginlib as plugins will be used
ament_target_dependencies(
  area_calculator
  "pluginlib"
)

# Consider the files in the include directory 
install(
  DIRECTORY include/
  DESTINATION include
)

# Add executables to library destination
install(TARGETS area_calculator
  DESTINATION lib/${PROJECT_NAME})

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

# -------------------- CONSTRUCT PACKAGE WITH AMENT CMAKE ----------------------
ament_package()
