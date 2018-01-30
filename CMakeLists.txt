cmake_minimum_required(VERSION 2.7)
project(robocon)

set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
set(CMAKE_C_FLAGS_RELEASE  "${CMAKE_C_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -pg -march=native")
set(CMAKE_CXX_FLAGS_RELEASE   "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG    "${CMAKE_CXX_FLAGS_DEBUG} -Wall -pg -march=native")

find_package(OpenCV 3.1.0 REQUIRED)
find_package(Boost  REQUIRED)
find_package(OpenNI2 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}/trace
        ${PROJECT_SOURCE_DIR}/cameraLocation
        ${PROJECT_SOURCE_DIR}/radarLocation
        ${PROJECT_SOURCE_DIR}/protocol
        ${PROJECT_SOURCE_DIR}/apriltag
        ${OPENCV_INCLUDE_DIRS}
        ${OPENNI2_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/trace)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/cameraLocation)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/radarLocation)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/thread)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/protocol)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/apriltag)

set(SOURCE_FILES calibration/floor_fit_cloud.cpp)
add_executable(robocon ${SOURCE_FILES})
target_link_libraries(robocon
        thread
        ${BOOST_LIBRARIES}
        )