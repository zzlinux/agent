set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(thread
        ThreadController.cpp
        ThreadController.h
        Param.cpp
        )
target_link_libraries(thread
        trace
        camera
        protocol
        radar
        apriltag
        )
