cmake_minimum_required(VERSION 3.0)
project(easygopigo3_cpp)

### Build

# set (EXECUTABLE_OUPUT_PATH bin)
set (CMAKE_BINARY_DIR build)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include_directories(include)

## GoPiGo3 library
add_library(gopigo3 SHARED src/GoPiGo3.cpp)
add_library(easygopigo3 SHARED src/EasyGoPiGo3.cpp)




# test program
add_executable(egpgtest test/egpgtest.cpp)
target_link_libraries(egpgtest gopigo3 easygopigo3)

# Clean
add_custom_target(clean-cmake-files
   COMMAND ${CMAKE_COMMAND} -P clean-all.cmake
)

# clean-all.cmake
set(cmake_generated ${CMAKE_BINARY_DIR}/CMakeCache.txt
                    ${CMAKE_BINARY_DIR}/cmake_install.cmake
                    ${CMAKE_BINARY_DIR}/Makefile
                    ${CMAKE_BINARY_DIR}/CMakeFiles
)

foreach(file ${cmake_generated})

  if (EXISTS ${file})
     file(REMOVE_RECURSE ${file})
  endif()

endforeach(file)


add_custom_target(clean-all
   COMMAND ${CMAKE_BUILD_TOOL} clean
   COMMAND ${CMAKE_COMMAND} -P clean-all.cmake
)

### Installation

