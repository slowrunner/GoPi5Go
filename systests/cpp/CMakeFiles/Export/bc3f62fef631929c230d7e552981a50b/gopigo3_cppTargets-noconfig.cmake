#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gopigo3" for configuration ""
set_property(TARGET gopigo3 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(gopigo3 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libgopigo3.so"
  IMPORTED_SONAME_NOCONFIG "libgopigo3.so"
  )

list(APPEND _cmake_import_check_targets gopigo3 )
list(APPEND _cmake_import_check_files_for_gopigo3 "${_IMPORT_PREFIX}/lib/libgopigo3.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
