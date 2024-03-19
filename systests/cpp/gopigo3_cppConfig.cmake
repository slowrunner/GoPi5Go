# - Config file for the GOPIGO3 package
# It defines the following variables
#  GOPIGO3_INCLUDE_DIRS - include directories for gopigo3_cpp
#  GOPIGO3_LIBRARIES    - libraries to link against

# Compute paths
get_filename_component(GOPIGO3_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(GOPIGO3_INCLUDE_DIRS "/usr/local/include/")

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET gopigo3 AND NOT GOPIGO3_BINARY_DIR)
  include("${GOPIGO3_CMAKE_DIR}/gopigo3_cppTargets.cmake")
endif()

# These are IMPORTED targets created by gopigo3_cppTargets.cmake
set(GOPIGO3_LIBRARIES gopigo3)
