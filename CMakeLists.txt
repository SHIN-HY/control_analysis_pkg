cmake_minimum_required(VERSION 3.8)
project(control_analysis_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(
  control_analysis_component SHARED
  src/control_analysis_component.cpp
)

ament_target_dependencies(
  control_analysis_component
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_nodes(
  control_analysis_component
  "control_analysis_component_ns::ControlAnalysis"
)

install(TARGETS 
  control_analysis_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
