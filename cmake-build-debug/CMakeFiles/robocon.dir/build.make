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
include CMakeFiles/robocon.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/robocon.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/robocon.dir/flags.make

CMakeFiles/robocon.dir/main.cpp.o: CMakeFiles/robocon.dir/flags.make
CMakeFiles/robocon.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/robocon.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robocon.dir/main.cpp.o -c /home/robocon/workspace/agent/main.cpp

CMakeFiles/robocon.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robocon.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent/main.cpp > CMakeFiles/robocon.dir/main.cpp.i

CMakeFiles/robocon.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robocon.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent/main.cpp -o CMakeFiles/robocon.dir/main.cpp.s

CMakeFiles/robocon.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/robocon.dir/main.cpp.o.requires

CMakeFiles/robocon.dir/main.cpp.o.provides: CMakeFiles/robocon.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/robocon.dir/build.make CMakeFiles/robocon.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/robocon.dir/main.cpp.o.provides

CMakeFiles/robocon.dir/main.cpp.o.provides.build: CMakeFiles/robocon.dir/main.cpp.o


# Object files for target robocon
robocon_OBJECTS = \
"CMakeFiles/robocon.dir/main.cpp.o"

# External object files for target robocon
robocon_EXTERNAL_OBJECTS =

../bin/robocon: CMakeFiles/robocon.dir/main.cpp.o
../bin/robocon: CMakeFiles/robocon.dir/build.make
../bin/robocon: thread/libthread.a
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/robocon: trace/libtrace.a
../bin/robocon: /usr/local/lib/libpcl_common.so
../bin/robocon: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/robocon: /usr/lib/libvtkGeovis.so.5.8.0
../bin/robocon: /usr/lib/libvtkCharts.so.5.8.0
../bin/robocon: /usr/lib/libvtkViews.so.5.8.0
../bin/robocon: /usr/lib/libvtkInfovis.so.5.8.0
../bin/robocon: /usr/lib/libvtkWidgets.so.5.8.0
../bin/robocon: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/robocon: /usr/lib/libvtkHybrid.so.5.8.0
../bin/robocon: /usr/lib/libvtkParallel.so.5.8.0
../bin/robocon: /usr/lib/libvtkRendering.so.5.8.0
../bin/robocon: /usr/lib/libvtkImaging.so.5.8.0
../bin/robocon: /usr/lib/libvtkGraphics.so.5.8.0
../bin/robocon: /usr/lib/libvtkIO.so.5.8.0
../bin/robocon: /usr/lib/libvtkFiltering.so.5.8.0
../bin/robocon: /usr/lib/libvtkCommon.so.5.8.0
../bin/robocon: /usr/lib/libvtksys.so.5.8.0
../bin/robocon: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/robocon: /usr/local/lib/libpcl_kdtree.so
../bin/robocon: /usr/local/lib/libpcl_octree.so
../bin/robocon: /usr/local/lib/libpcl_search.so
../bin/robocon: /usr/local/lib/libpcl_sample_consensus.so
../bin/robocon: /usr/local/lib/libpcl_filters.so
../bin/robocon: /usr/lib/libOpenNI2.so
../bin/robocon: /usr/local/lib/libpcl_io.so
../bin/robocon: /usr/local/lib/libpcl_features.so
../bin/robocon: /usr/local/lib/libpcl_visualization.so
../bin/robocon: /usr/local/lib/libpcl_ml.so
../bin/robocon: /usr/local/lib/libpcl_segmentation.so
../bin/robocon: /usr/local/lib/libpcl_people.so
../bin/robocon: /usr/local/lib/libpcl_registration.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/robocon: /usr/local/lib/libpcl_surface.so
../bin/robocon: /usr/local/lib/libpcl_tracking.so
../bin/robocon: /usr/local/lib/libpcl_outofcore.so
../bin/robocon: /usr/local/lib/libpcl_keypoints.so
../bin/robocon: /usr/local/lib/libpcl_stereo.so
../bin/robocon: /usr/local/lib/libpcl_recognition.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/robocon: /usr/local/lib/libpcl_kdtree.so
../bin/robocon: /usr/local/lib/libpcl_octree.so
../bin/robocon: /usr/local/lib/libpcl_search.so
../bin/robocon: /usr/local/lib/libpcl_sample_consensus.so
../bin/robocon: /usr/local/lib/libpcl_filters.so
../bin/robocon: /usr/lib/libOpenNI2.so
../bin/robocon: /usr/local/lib/libpcl_io.so
../bin/robocon: /usr/local/lib/libpcl_features.so
../bin/robocon: /usr/local/lib/libpcl_visualization.so
../bin/robocon: /usr/local/lib/libpcl_ml.so
../bin/robocon: /usr/local/lib/libpcl_segmentation.so
../bin/robocon: /usr/local/lib/libpcl_people.so
../bin/robocon: /usr/local/lib/libpcl_registration.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/robocon: /usr/local/lib/libpcl_surface.so
../bin/robocon: /usr/local/lib/libpcl_tracking.so
../bin/robocon: /usr/local/lib/libpcl_outofcore.so
../bin/robocon: /usr/local/lib/libpcl_keypoints.so
../bin/robocon: /usr/local/lib/libpcl_stereo.so
../bin/robocon: /usr/local/lib/libpcl_recognition.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/robocon: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/robocon: cameraLocation/libcamera.a
../bin/robocon: protocol/libprotocol.a
../bin/robocon: radarLocation/libradar.a
../bin/robocon: /usr/local/lib/libopencv_videostab.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_superres.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_stitching.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_shape.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_video.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_photo.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_objdetect.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_calib3d.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_features2d.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_ml.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_highgui.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_videoio.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_imgproc.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_flann.so.3.1.0
../bin/robocon: /usr/local/lib/libopencv_core.so.3.1.0
../bin/robocon: CMakeFiles/robocon.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robocon/workspace/agent/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/robocon"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robocon.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/robocon.dir/build: ../bin/robocon

.PHONY : CMakeFiles/robocon.dir/build

CMakeFiles/robocon.dir/requires: CMakeFiles/robocon.dir/main.cpp.o.requires

.PHONY : CMakeFiles/robocon.dir/requires

CMakeFiles/robocon.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/robocon.dir/cmake_clean.cmake
.PHONY : CMakeFiles/robocon.dir/clean

CMakeFiles/robocon.dir/depend:
	cd /home/robocon/workspace/agent/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocon/workspace/agent /home/robocon/workspace/agent /home/robocon/workspace/agent/cmake-build-debug /home/robocon/workspace/agent/cmake-build-debug /home/robocon/workspace/agent/cmake-build-debug/CMakeFiles/robocon.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/robocon.dir/depend

