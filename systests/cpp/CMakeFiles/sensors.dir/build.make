# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/GoPi5Go/systests/cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/GoPi5Go/systests/cpp

# Include any dependencies generated for this target.
include CMakeFiles/sensors.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/sensors.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/sensors.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sensors.dir/flags.make

CMakeFiles/sensors.dir/Examples/sensors.cpp.o: CMakeFiles/sensors.dir/flags.make
CMakeFiles/sensors.dir/Examples/sensors.cpp.o: Examples/sensors.cpp
CMakeFiles/sensors.dir/Examples/sensors.cpp.o: CMakeFiles/sensors.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/GoPi5Go/systests/cpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sensors.dir/Examples/sensors.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/sensors.dir/Examples/sensors.cpp.o -MF CMakeFiles/sensors.dir/Examples/sensors.cpp.o.d -o CMakeFiles/sensors.dir/Examples/sensors.cpp.o -c /home/pi/GoPi5Go/systests/cpp/Examples/sensors.cpp

CMakeFiles/sensors.dir/Examples/sensors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensors.dir/Examples/sensors.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/GoPi5Go/systests/cpp/Examples/sensors.cpp > CMakeFiles/sensors.dir/Examples/sensors.cpp.i

CMakeFiles/sensors.dir/Examples/sensors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensors.dir/Examples/sensors.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/GoPi5Go/systests/cpp/Examples/sensors.cpp -o CMakeFiles/sensors.dir/Examples/sensors.cpp.s

# Object files for target sensors
sensors_OBJECTS = \
"CMakeFiles/sensors.dir/Examples/sensors.cpp.o"

# External object files for target sensors
sensors_EXTERNAL_OBJECTS =

sensors: CMakeFiles/sensors.dir/Examples/sensors.cpp.o
sensors: CMakeFiles/sensors.dir/build.make
sensors: libgopigo3.so
sensors: CMakeFiles/sensors.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/GoPi5Go/systests/cpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sensors"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sensors.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sensors.dir/build: sensors
.PHONY : CMakeFiles/sensors.dir/build

CMakeFiles/sensors.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sensors.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sensors.dir/clean

CMakeFiles/sensors.dir/depend:
	cd /home/pi/GoPi5Go/systests/cpp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/GoPi5Go/systests/cpp /home/pi/GoPi5Go/systests/cpp /home/pi/GoPi5Go/systests/cpp /home/pi/GoPi5Go/systests/cpp /home/pi/GoPi5Go/systests/cpp/CMakeFiles/sensors.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sensors.dir/depend

