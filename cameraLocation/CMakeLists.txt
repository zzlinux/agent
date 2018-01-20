set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(camera
        CameraController.cpp
        CameraModel.cpp
        )
target_link_libraries(camera
        ${OpenCV_LIBS}
        )