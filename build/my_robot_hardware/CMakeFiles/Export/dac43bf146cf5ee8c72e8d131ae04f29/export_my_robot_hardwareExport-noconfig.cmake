#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "my_robot_hardware::pca9685_bts7960" for configuration ""
set_property(TARGET my_robot_hardware::pca9685_bts7960 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(my_robot_hardware::pca9685_bts7960 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpca9685_bts7960.so"
  IMPORTED_SONAME_NOCONFIG "libpca9685_bts7960.so"
  )

list(APPEND _cmake_import_check_targets my_robot_hardware::pca9685_bts7960 )
list(APPEND _cmake_import_check_files_for_my_robot_hardware::pca9685_bts7960 "${_IMPORT_PREFIX}/lib/libpca9685_bts7960.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
