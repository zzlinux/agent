# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/robocon/Documents/software/clion-2017.2.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/robocon/Documents/software/clion-2017.2.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robocon/workspace/agent

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robocon/workspace/agent/cmake-build-debug

# Include any dependencies generated for this target.
include cameraLocation/CMakeFiles/camera.dir/depend.make

# Include the progress variables for this target.
include cameraLocation/CMakeFiles/camera.dir/progress.make

# Include the compile flags for this target's objects.
include cameraLocation/CMakeFiles/camera.dir/flags.make

cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o: cameraLocation/CMakeFiles/camera.dir/flags.make
cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o: ../cameraLocation/CameraController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o"
	cd /home/robocon/workspace/agent/cmake-build-debug/cameraLocation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/CameraController.cpp.o -c /home/robocon/workspace/agent/cameraLocation/CameraController.cpp

cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/CameraController.cpp.i"
	cd /home/robocon/workspace/agent/cmake-build-debug/cameraLocation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent/cameraLocation/CameraController.cpp > CMakeFiles/camera.dir/CameraController.cpp.i

cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/CameraController.cpp.s"
	cd /home/robocon/workspace/agent/cmake-build-debug/cameraLocation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent/cameraLocation/CameraController.cpp -o CMakeFiles/camera.dir/CameraController.cpp.s

cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o.requires:

.PHONY : cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o.requires

cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o.provides: cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o.requires
	$(MAKE) -f cameraLocation/CMakeFiles/camera.dir/build.make cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o.provides.build
.PHONY : cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o.provides

cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o.provides.build: cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o


cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o: cameraLocation/CMakeFiles/camera.dir/flags.make
cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o: ../cameraLocation/CameraModel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o"
	cd /home/robocon/workspace/agent/cmake-build-debug/cameraLocation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/CameraModel.cpp.o -c /home/robocon/workspace/agent/cameraLocation/CameraModel.cpp

cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/CameraModel.cpp.i"
	cd /home/robocon/workspace/agent/cmake-build-debug/cameraLocation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent/cameraLocation/CameraModel.cpp > CMakeFiles/camera.dir/CameraModel.cpp.i

cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/CameraModel.cpp.s"
	cd /home/robocon/workspace/agent/cmake-build-debug/cameraLocation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent/cameraLocation/CameraModel.cpp -o CMakeFiles/camera.dir/CameraModel.cpp.s

cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o.requires:

.PHONY : cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o.requires

cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o.provides: cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o.requires
	$(MAKE) -f cameraLocation/CMakeFiles/camera.dir/build.make cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o.provides.build
.PHONY : cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o.provides

cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o.provides.build: cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o


# Object files for target camera
camera_OBJECTS = \
"CMakeFiles/camera.dir/CameraController.cpp.o" \
"CMakeFiles/camera.dir/CameraModel.cpp.o"

# External object files for target camera
camera_EXTERNAL_OBJECTS =

cameraLocation/libcamera.a: cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o
cameraLocation/libcamera.a: cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o
cameraLocation/libcamera.a: cameraLocation/CMakeFiles/camera.dir/build.make
cameraLocation/libcamera.a: cameraLocation/CMakeFiles/camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robocon/workspace/agent/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libcamera.a"
	cd /home/robocon/workspace/agent/cmake-build-debug/cameraLocation && $(CMAKE_COMMAND) -P CMakeFiles/camera.dir/cmake_clean_target.cmake
	cd /home/robocon/workspace/agent/cmake-build-debug/cameraLocation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cameraLocation/CMakeFiles/camera.dir/build: cameraLocation/libcamera.a

.PHONY : cameraLocation/CMakeFiles/camera.dir/build

cameraLocation/CMakeFiles/camera.dir/requires: cameraLocation/CMakeFiles/camera.dir/CameraController.cpp.o.requires
cameraLocation/CMakeFiles/camera.dir/requires: cameraLocation/CMakeFiles/camera.dir/CameraModel.cpp.o.requires

.PHONY : cameraLocation/CMakeFiles/camera.dir/requires

cameraLocation/CMakeFiles/camera.dir/clean:
	cd /home/robocon/workspace/agent/cmake-build-debug/cameraLocation && $(CMAKE_COMMAND) -P CMakeFiles/camera.dir/cmake_clean.cmake
.PHONY : cameraLocation/CMakeFiles/camera.dir/clean

cameraLocation/CMakeFiles/camera.dir/depend:
	cd /home/robocon/workspace/agent/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocon/workspace/agent /home/robocon/workspace/agent/cameraLocation /home/robocon/workspace/agent/cmake-build-debug /home/robocon/workspace/agent/cmake-build-debug/cameraLocation /home/robocon/workspace/agent/cmake-build-debug/cameraLocation/CMakeFiles/camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cameraLocation/CMakeFiles/camera.dir/depend

