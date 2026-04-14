#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "manipulator::manipulator_core" for configuration ""
set_property(TARGET manipulator::manipulator_core APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(manipulator::manipulator_core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmanipulator_core.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS manipulator::manipulator_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_manipulator::manipulator_core "${_IMPORT_PREFIX}/lib/libmanipulator_core.a" )

# Import target "manipulator::gravity_compensation" for configuration ""
set_property(TARGET manipulator::gravity_compensation APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(manipulator::gravity_compensation PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libgravity_compensation.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS manipulator::gravity_compensation )
list(APPEND _IMPORT_CHECK_FILES_FOR_manipulator::gravity_compensation "${_IMPORT_PREFIX}/lib/libgravity_compensation.a" )

# Import target "manipulator::gravity_compensation_pinocchio" for configuration ""
set_property(TARGET manipulator::gravity_compensation_pinocchio APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(manipulator::gravity_compensation_pinocchio PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libgravity_compensation_pinocchio.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS manipulator::gravity_compensation_pinocchio )
list(APPEND _IMPORT_CHECK_FILES_FOR_manipulator::gravity_compensation_pinocchio "${_IMPORT_PREFIX}/lib/libgravity_compensation_pinocchio.a" )

# Import target "manipulator::arm_hardware_node" for configuration ""
set_property(TARGET manipulator::arm_hardware_node APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(manipulator::arm_hardware_node PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/manipulator/arm_hardware_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS manipulator::arm_hardware_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_manipulator::arm_hardware_node "${_IMPORT_PREFIX}/lib/manipulator/arm_hardware_node" )

# Import target "manipulator::master_arm_node" for configuration ""
set_property(TARGET manipulator::master_arm_node APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(manipulator::master_arm_node PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/manipulator/master_arm_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS manipulator::master_arm_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_manipulator::master_arm_node "${_IMPORT_PREFIX}/lib/manipulator/master_arm_node" )

# Import target "manipulator::slave_arm_node" for configuration ""
set_property(TARGET manipulator::slave_arm_node APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(manipulator::slave_arm_node PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/manipulator/slave_arm_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS manipulator::slave_arm_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_manipulator::slave_arm_node "${_IMPORT_PREFIX}/lib/manipulator/slave_arm_node" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
