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
  src/arm/abs_arm.cc
  src/arm/a_l1_beta.cc
  src/arm/a_l1_gamma.cc
  src/arm/arm_factory.cc
  src/bus/abs_bus.cc
  src/bus/serial_bus.cc
  src/motor/dm_motor.cc
  src/protocol/abs_protocol.cc
  src/protocol/protocol_factory.cc
  src/protocol/protocol_v1.cc
)

add_library(gravity_compensation
  src/gravity_compensation.cc
)

target_link_libraries(gravity_compensation
  Eigen3::Eigen
)

add_library(gravity_compensation_pinocchio
  src/gravity_compensation_pinocchio.cc
)

target_link_libraries(gravity_compensation_pinocchio
  Eigen3::Eigen
  pinocchio::pinocchio
)

target_include_directories(manipulator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(gravity_compensation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(gravity_compensation_pinocchio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(manipulator_core
  rclcpp
  std_msgs
  dummy_interface
  serial
  sensor_msgs
)

ament_target_dependencies(gravity_compensation
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
)

ament_target_dependencies(gravity_compensation_pinocchio
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
)

# ==============================
# Robot Arm Node (ROS2 Node)
# ==============================
add_executable(arm_hardware_node
  src/arm_hardware_node.cc
)

target_link_libraries(arm_hardware_node
  manipulator_core
)

add_executable(master_arm_node
  src/master_arm_node.cc
)

target_link_libraries(master_arm_node
  manipulator_core
  gravity_compensation_pinocchio
)

ament_target_dependencies(master_arm_node
  rclcpp
  sensor_msgs
  std_msgs
  dummy_interface
  geometry_msgs
)

add_executable(slave_arm_node
  src/slave_arm_node.cc
)

target_link_libraries(slave_arm_node
  manipulator_core
  gravity_compensation_pinocchio
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
  gravity_compensation
  gravity_compensation_pinocchio
  arm_hardware_node
  master_arm_node
  slave_arm_node
  # moveit_bridge_node
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