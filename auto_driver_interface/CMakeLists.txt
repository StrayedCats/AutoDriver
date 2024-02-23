cmake_minimum_required(VERSION 3.5)
project(auto_driver_interface)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# yaw_node --------------------------------------------------
set(TARGET pid_node)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${TARGET}.cpp)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::PidNode"
  EXECUTABLE ${PROJECT_NAME}_exec)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
