cmake_minimum_required(VERSION 3.8)
project(manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================
# ROS2 dependencies
# ==============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dummy_interface REQUIRED)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# ==============================
# Arm Platform Core Library
# ==============================
add_library(manipulator_core
  src/robotics/arm_platform.cc
  src/robotics/arm/abs_arm.cc
  src/robotics/arm/a_l1_beta.cc
  src/robotics/arm/a_l1_gamma.cc
  src/robotics/arm/arm_factory.cc
  src/robotics/bus/abs_bus.cc
  src/robotics/bus/serial_bus.cc
  src/robotics/motor/dm_motor.cc
  src/robotics/protocol/abs_protocol.cc
  src/robotics/protocol/protocol_factory.cc
  src/robotics/protocol/protocol_v1.cc
)

add_library(controller_lib
  src/controller/smooth_position_controller.cc
  src/controller/gravity_controller.cc
)

add_library(planner_lib
  src/planning/abs_motion_planner.cc
  src/planning/reset_motion_planner.cc
)

ament_target_dependencies(planner_lib
  rclcpp
  std_msgs
  sensor_msgs
  dummy_interface
)

ament_target_dependencies(controller_lib
  rclcpp
  geometry_msgs
  dummy_interface
)

target_link_libraries(planner_lib
  Eigen3::Eigen
)

target_link_libraries(controller_lib
  Eigen3::Eigen
  pinocchio::pinocchio
)

target_link_libraries(manipulator_core
  Eigen3::Eigen
)

target_include_directories(manipulator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ==============================
# Robot Arm Node (ROS2 Node)
# ==============================
add_executable(arm_hardware_node
  src/ros_nodes/arm_hardware_node.cc
)

target_link_libraries(arm_hardware_node
  manipulator_core
  controller_lib
  planner_lib
)

add_executable(master_arm_node
  src/ros_nodes/master_arm_node.cc
)

target_link_libraries(master_arm_node
  manipulator_core
  controller_lib
  planner_lib
)

ament_target_dependencies(manipulator_core
  rclcpp
  std_msgs
  serial
  sensor_msgs
  dummy_interface
)

ament_target_dependencies(master_arm_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  dummy_interface
)

add_executable(slave_arm_node
  src/ros_nodes/slave_arm_node.cc
)

target_link_libraries(slave_arm_node
  manipulator_core
  controller_lib
  planner_lib
)

ament_target_dependencies(slave_arm_node
  rclcpp
  sensor_msgs
  std_msgs
  dummy_interface
)

# ==============================
# Install
# ==============================
install(TARGETS
  manipulator_core
  arm_hardware_node
  master_arm_node
  slave_arm_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/
)

# ==============================
# Export
# ==============================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs std_msgs dummy_interface)

# ==============================
# Tests & Lint
# ==============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()