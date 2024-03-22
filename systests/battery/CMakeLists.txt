cmake_minimum_required(VERSION 3.0)
project(batteryLife_cpp)

### Build

set (CMAKE_BINARY_DIR ./build)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
include_directories(include)

## GoPiGo3 library
add_library(gopigo3 SHARED src/GoPiGo3.cpp)
add_library(easygopigo3 SHARED src/EasyGoPiGo3.cpp)




# batteryLife
add_executable(batteryLife src/batteryLife.cpp)
target_link_libraries(batteryLife gopigo3 easygopigo3)


### Installation

