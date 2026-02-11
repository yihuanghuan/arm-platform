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


# ==============================
# Arm Platform Core Library
# ==============================
add_library(manipulator_core
  src/arm/abs_arm.cc
  src/arm/a_l1_beta.cc
  src/bus/abs_bus.cc
  src/bus/serial_bus.cc
  src/motor/dm_motor.cc
  src/protocol/abs_protocol.cc
  src/protocol/protocol_factory.cc
  src/protocol/protocol_v1.cc
)

target_include_directories(manipulator_core PUBLIC
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

# ==============================
# Robot Arm Node (ROS2 Node)
# ==============================
add_executable(arm_hardware_node
  src/arm_hardware_node.cc
)

target_link_libraries(arm_hardware_node
  manipulator_core
)

# add_executable(moveit_bridge_node
#   src/moveit_bridge_node.cc
# )

# ament_target_dependencies(moveit_bridge_node
#   rclcpp
#   sensor_msgs
#   std_msgs
#   dummy_interface
# )

# target_include_directories(moveit_bridge_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )


# ==============================
# Install
# ==============================
install(TARGETS
  manipulator_core
  arm_hardware_node
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