#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hardware_unitree_sdk2::hardware_unitree_sdk2" for configuration ""
set_property(TARGET hardware_unitree_sdk2::hardware_unitree_sdk2 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(hardware_unitree_sdk2::hardware_unitree_sdk2 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libhardware_unitree_sdk2.so"
  IMPORTED_SONAME_NOCONFIG "libhardware_unitree_sdk2.so"
  )

list(APPEND _cmake_import_check_targets hardware_unitree_sdk2::hardware_unitree_sdk2 )
list(APPEND _cmake_import_check_files_for_hardware_unitree_sdk2::hardware_unitree_sdk2 "${_IMPORT_PREFIX}/lib/libhardware_unitree_sdk2.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
