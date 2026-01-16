cmake_minimum_required(VERSION 3.8)
project(arm_platform)

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
add_library(arm_platform_core
  src/controller/dummy_controller.cc
  src/protocol/dummy_link_protocol.cc
  src/bus/serial_bus.cc
)

target_include_directories(arm_platform_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arm_platform_core
  rclcpp
  sensor_msgs
  std_msgs
  dummy_interface
  serial
)

# ==============================
# Robot Arm Node (ROS2 Node)
# ==============================
add_executable(arm_hardware_node
  src/arm_hardware_node.cc
)

target_link_libraries(arm_hardware_node
  arm_platform_core
)

add_executable(moveit_bridge_node
  src/moveit_bridge_node.cc
)

ament_target_dependencies(moveit_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  dummy_interface
)

target_include_directories(moveit_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# ==============================
# Install
# ==============================
install(TARGETS
  arm_platform_core
  arm_hardware_node
  moveit_bridge_node
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