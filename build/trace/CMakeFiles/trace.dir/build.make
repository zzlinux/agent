# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robocon/workspace/CLionProjects/robocon

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robocon/workspace/CLionProjects/robocon/build

# Include any dependencies generated for this target.
include trace/CMakeFiles/trace.dir/depend.make

# Include the progress variables for this target.
include trace/CMakeFiles/trace.dir/progress.make

# Include the compile flags for this target's objects.
include trace/CMakeFiles/trace.dir/flags.make

trace/CMakeFiles/trace.dir/BallAssociate.cpp.o: trace/CMakeFiles/trace.dir/flags.make
trace/CMakeFiles/trace.dir/BallAssociate.cpp.o: ../trace/BallAssociate.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robocon/workspace/CLionProjects/robocon/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object trace/CMakeFiles/trace.dir/BallAssociate.cpp.o"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/BallAssociate.cpp.o -c /home/robocon/workspace/CLionProjects/robocon/trace/BallAssociate.cpp

trace/CMakeFiles/trace.dir/BallAssociate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/BallAssociate.cpp.i"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robocon/workspace/CLionProjects/robocon/trace/BallAssociate.cpp > CMakeFiles/trace.dir/BallAssociate.cpp.i

trace/CMakeFiles/trace.dir/BallAssociate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/BallAssociate.cpp.s"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robocon/workspace/CLionProjects/robocon/trace/BallAssociate.cpp -o CMakeFiles/trace.dir/BallAssociate.cpp.s

trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires:
.PHONY : trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires

trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides: trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires
	$(MAKE) -f trace/CMakeFiles/trace.dir/build.make trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides.build
.PHONY : trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides

trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides.build: trace/CMakeFiles/trace.dir/BallAssociate.cpp.o

trace/CMakeFiles/trace.dir/BallDetector.cpp.o: trace/CMakeFiles/trace.dir/flags.make
trace/CMakeFiles/trace.dir/BallDetector.cpp.o: ../trace/BallDetector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robocon/workspace/CLionProjects/robocon/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object trace/CMakeFiles/trace.dir/BallDetector.cpp.o"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/BallDetector.cpp.o -c /home/robocon/workspace/CLionProjects/robocon/trace/BallDetector.cpp

trace/CMakeFiles/trace.dir/BallDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/BallDetector.cpp.i"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robocon/workspace/CLionProjects/robocon/trace/BallDetector.cpp > CMakeFiles/trace.dir/BallDetector.cpp.i

trace/CMakeFiles/trace.dir/BallDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/BallDetector.cpp.s"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robocon/workspace/CLionProjects/robocon/trace/BallDetector.cpp -o CMakeFiles/trace.dir/BallDetector.cpp.s

trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires:
.PHONY : trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires

trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides: trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires
	$(MAKE) -f trace/CMakeFiles/trace.dir/build.make trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides.build
.PHONY : trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides

trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides.build: trace/CMakeFiles/trace.dir/BallDetector.cpp.o

trace/CMakeFiles/trace.dir/CircleDetector.cpp.o: trace/CMakeFiles/trace.dir/flags.make
trace/CMakeFiles/trace.dir/CircleDetector.cpp.o: ../trace/CircleDetector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robocon/workspace/CLionProjects/robocon/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object trace/CMakeFiles/trace.dir/CircleDetector.cpp.o"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/CircleDetector.cpp.o -c /home/robocon/workspace/CLionProjects/robocon/trace/CircleDetector.cpp

trace/CMakeFiles/trace.dir/CircleDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/CircleDetector.cpp.i"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robocon/workspace/CLionProjects/robocon/trace/CircleDetector.cpp > CMakeFiles/trace.dir/CircleDetector.cpp.i

trace/CMakeFiles/trace.dir/CircleDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/CircleDetector.cpp.s"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robocon/workspace/CLionProjects/robocon/trace/CircleDetector.cpp -o CMakeFiles/trace.dir/CircleDetector.cpp.s

trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires:
.PHONY : trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires

trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides: trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires
	$(MAKE) -f trace/CMakeFiles/trace.dir/build.make trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides.build
.PHONY : trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides

trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides.build: trace/CMakeFiles/trace.dir/CircleDetector.cpp.o

trace/CMakeFiles/trace.dir/FitTrace.cpp.o: trace/CMakeFiles/trace.dir/flags.make
trace/CMakeFiles/trace.dir/FitTrace.cpp.o: ../trace/FitTrace.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robocon/workspace/CLionProjects/robocon/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object trace/CMakeFiles/trace.dir/FitTrace.cpp.o"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/FitTrace.cpp.o -c /home/robocon/workspace/CLionProjects/robocon/trace/FitTrace.cpp

trace/CMakeFiles/trace.dir/FitTrace.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/FitTrace.cpp.i"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robocon/workspace/CLionProjects/robocon/trace/FitTrace.cpp > CMakeFiles/trace.dir/FitTrace.cpp.i

trace/CMakeFiles/trace.dir/FitTrace.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/FitTrace.cpp.s"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robocon/workspace/CLionProjects/robocon/trace/FitTrace.cpp -o CMakeFiles/trace.dir/FitTrace.cpp.s

trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires:
.PHONY : trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires

trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides: trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires
	$(MAKE) -f trace/CMakeFiles/trace.dir/build.make trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides.build
.PHONY : trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides

trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides.build: trace/CMakeFiles/trace.dir/FitTrace.cpp.o

trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o: trace/CMakeFiles/trace.dir/flags.make
trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o: ../trace/rgbd_camera.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robocon/workspace/CLionProjects/robocon/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/rgbd_camera.cpp.o -c /home/robocon/workspace/CLionProjects/robocon/trace/rgbd_camera.cpp

trace/CMakeFiles/trace.dir/rgbd_camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/rgbd_camera.cpp.i"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robocon/workspace/CLionProjects/robocon/trace/rgbd_camera.cpp > CMakeFiles/trace.dir/rgbd_camera.cpp.i

trace/CMakeFiles/trace.dir/rgbd_camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/rgbd_camera.cpp.s"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robocon/workspace/CLionProjects/robocon/trace/rgbd_camera.cpp -o CMakeFiles/trace.dir/rgbd_camera.cpp.s

trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires:
.PHONY : trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires

trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides: trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires
	$(MAKE) -f trace/CMakeFiles/trace.dir/build.make trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides.build
.PHONY : trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides

trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides.build: trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o

trace/CMakeFiles/trace.dir/Trajectory.cpp.o: trace/CMakeFiles/trace.dir/flags.make
trace/CMakeFiles/trace.dir/Trajectory.cpp.o: ../trace/Trajectory.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robocon/workspace/CLionProjects/robocon/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object trace/CMakeFiles/trace.dir/Trajectory.cpp.o"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/Trajectory.cpp.o -c /home/robocon/workspace/CLionProjects/robocon/trace/Trajectory.cpp

trace/CMakeFiles/trace.dir/Trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/Trajectory.cpp.i"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robocon/workspace/CLionProjects/robocon/trace/Trajectory.cpp > CMakeFiles/trace.dir/Trajectory.cpp.i

trace/CMakeFiles/trace.dir/Trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/Trajectory.cpp.s"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robocon/workspace/CLionProjects/robocon/trace/Trajectory.cpp -o CMakeFiles/trace.dir/Trajectory.cpp.s

trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires:
.PHONY : trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires

trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides: trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires
	$(MAKE) -f trace/CMakeFiles/trace.dir/build.make trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides.build
.PHONY : trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides

trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides.build: trace/CMakeFiles/trace.dir/Trajectory.cpp.o

trace/CMakeFiles/trace.dir/transformer.cpp.o: trace/CMakeFiles/trace.dir/flags.make
trace/CMakeFiles/trace.dir/transformer.cpp.o: ../trace/transformer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robocon/workspace/CLionProjects/robocon/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object trace/CMakeFiles/trace.dir/transformer.cpp.o"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/transformer.cpp.o -c /home/robocon/workspace/CLionProjects/robocon/trace/transformer.cpp

trace/CMakeFiles/trace.dir/transformer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/transformer.cpp.i"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robocon/workspace/CLionProjects/robocon/trace/transformer.cpp > CMakeFiles/trace.dir/transformer.cpp.i

trace/CMakeFiles/trace.dir/transformer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/transformer.cpp.s"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robocon/workspace/CLionProjects/robocon/trace/transformer.cpp -o CMakeFiles/trace.dir/transformer.cpp.s

trace/CMakeFiles/trace.dir/transformer.cpp.o.requires:
.PHONY : trace/CMakeFiles/trace.dir/transformer.cpp.o.requires

trace/CMakeFiles/trace.dir/transformer.cpp.o.provides: trace/CMakeFiles/trace.dir/transformer.cpp.o.requires
	$(MAKE) -f trace/CMakeFiles/trace.dir/build.make trace/CMakeFiles/trace.dir/transformer.cpp.o.provides.build
.PHONY : trace/CMakeFiles/trace.dir/transformer.cpp.o.provides

trace/CMakeFiles/trace.dir/transformer.cpp.o.provides.build: trace/CMakeFiles/trace.dir/transformer.cpp.o

# Object files for target trace
trace_OBJECTS = \
"CMakeFiles/trace.dir/BallAssociate.cpp.o" \
"CMakeFiles/trace.dir/BallDetector.cpp.o" \
"CMakeFiles/trace.dir/CircleDetector.cpp.o" \
"CMakeFiles/trace.dir/FitTrace.cpp.o" \
"CMakeFiles/trace.dir/rgbd_camera.cpp.o" \
"CMakeFiles/trace.dir/Trajectory.cpp.o" \
"CMakeFiles/trace.dir/transformer.cpp.o"

# External object files for target trace
trace_EXTERNAL_OBJECTS =

trace/libtrace.a: trace/CMakeFiles/trace.dir/BallAssociate.cpp.o
trace/libtrace.a: trace/CMakeFiles/trace.dir/BallDetector.cpp.o
trace/libtrace.a: trace/CMakeFiles/trace.dir/CircleDetector.cpp.o
trace/libtrace.a: trace/CMakeFiles/trace.dir/FitTrace.cpp.o
trace/libtrace.a: trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o
trace/libtrace.a: trace/CMakeFiles/trace.dir/Trajectory.cpp.o
trace/libtrace.a: trace/CMakeFiles/trace.dir/transformer.cpp.o
trace/libtrace.a: trace/CMakeFiles/trace.dir/build.make
trace/libtrace.a: trace/CMakeFiles/trace.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libtrace.a"
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && $(CMAKE_COMMAND) -P CMakeFiles/trace.dir/cmake_clean_target.cmake
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/trace.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
trace/CMakeFiles/trace.dir/build: trace/libtrace.a
.PHONY : trace/CMakeFiles/trace.dir/build

trace/CMakeFiles/trace.dir/requires: trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires
trace/CMakeFiles/trace.dir/requires: trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires
trace/CMakeFiles/trace.dir/requires: trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires
trace/CMakeFiles/trace.dir/requires: trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires
trace/CMakeFiles/trace.dir/requires: trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires
trace/CMakeFiles/trace.dir/requires: trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires
trace/CMakeFiles/trace.dir/requires: trace/CMakeFiles/trace.dir/transformer.cpp.o.requires
.PHONY : trace/CMakeFiles/trace.dir/requires

trace/CMakeFiles/trace.dir/clean:
	cd /home/robocon/workspace/CLionProjects/robocon/build/trace && $(CMAKE_COMMAND) -P CMakeFiles/trace.dir/cmake_clean.cmake
.PHONY : trace/CMakeFiles/trace.dir/clean

trace/CMakeFiles/trace.dir/depend:
	cd /home/robocon/workspace/CLionProjects/robocon/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocon/workspace/CLionProjects/robocon /home/robocon/workspace/CLionProjects/robocon/trace /home/robocon/workspace/CLionProjects/robocon/build /home/robocon/workspace/CLionProjects/robocon/build/trace /home/robocon/workspace/CLionProjects/robocon/build/trace/CMakeFiles/trace.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : trace/CMakeFiles/trace.dir/depend

