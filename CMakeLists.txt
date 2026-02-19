cmake_minimum_required(VERSION 3.8)
project(control_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# examples/offboard/offboard_node
# add_executable(offboard_node src/offboard_node.cpp)
# ament_target_dependencies(offboard_node rclcpp px4_msgs)
# install(TARGETS offboard_node DESTINATION lib/${PROJECT_NAME})

# add_executable(root_controller src/root_controller.cpp)
# ament_target_dependencies(root_controller rclcpp px4_msgs)
# install(TARGETS root_controller DESTINATION lib/${PROJECT_NAME})

add_executable(root_controller
  src/root_controller.cpp
  src/offboard_node.cpp   # <-- add this
)
ament_target_dependencies(root_controller rclcpp px4_msgs)
install(TARGETS root_controller DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control_srv
add_executable(offboard_control_srv src/offboard_control_srv.cpp)
ament_target_dependencies(offboard_control_srv rclcpp px4_msgs)
install(TARGETS offboard_control_srv DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_include_directories(include)
# ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


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

ament_package()
