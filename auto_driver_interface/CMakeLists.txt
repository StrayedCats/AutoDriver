cmake_minimum_required(VERSION 3.5)
project(auto_driver_interface)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# auto_driver_interface::PidNode
set(TARGET pid_node)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp)

rclcpp_components_register_node(
  ${TARGET}
  PLUGIN "${PROJECT_NAME}::PidNode"
  EXECUTABLE ${TARGET}_exec)

# auto_driver_interface::RollerNode
set(TARGET roller_node)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp)

rclcpp_components_register_node(
  ${TARGET}
  PLUGIN "${PROJECT_NAME}::RollerNode"
  EXECUTABLE ${TARGET}_exec)

# auto_driver_interface::HammerNode
set(TARGET hammer_node)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp)

rclcpp_components_register_node(
  ${TARGET}
  PLUGIN "${PROJECT_NAME}::HammerNode"
  EXECUTABLE ${TARGET}_exec)

# auto_driver_interface::TfToPositionActionServer
set(TARGET tf_to_position_server)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp)

rclcpp_components_register_node(
  ${TARGET}
  PLUGIN "${PROJECT_NAME}::TfToPositionActionServer"
  EXECUTABLE ${TARGET}_exec)

# auto_driver_interface::MoveToTargetDegServer
set(TARGET move_to_target_deg_server)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp)

rclcpp_components_register_node(
  ${TARGET}
  PLUGIN "${PROJECT_NAME}::MoveToTargetDegServer"
  EXECUTABLE ${TARGET}_exec)

ament_auto_package()
