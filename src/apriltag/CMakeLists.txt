set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(apriltag
        ApriltagController.cpp
        lib/Edge.h
        lib/FloatImage.h
        lib/Gaussian.h
        lib/GLine2D.h
        lib/GLineSegment2D.h
        lib/GrayModel.h
        lib/Gridder.h
        lib/Homography33.h
        lib/MathUtil.h
        lib/pch.h
        lib/Quad.h
        lib/Segment.h
        lib/Serial.h
        lib/Tag16h5.h
        lib/Tag16h5_other.h
        lib/Tag25h7.h
        lib/Tag25h9.h
        lib/Tag36h11.h
        lib/Tag36h11_other.h
        lib/Tag36h9.h
        lib/TagDetection.h
        lib/TagDetector.h
        lib/TagFamily.h
        lib/UnionFindSimple.h
        lib/XYWeight.h
        lib/Edge.cc
        lib/FloatImage.cc
        lib/Gaussian.cc
        lib/GLine2D.cc
        lib/GLineSegment2D.cc
        lib/GrayModel.cc
        lib/Homography33.cc
        lib/MathUtil.cc
        lib/Quad.cc
        lib/Segment.cc
        lib/Serial.cpp
        lib/TagDetection.cc
        lib/TagDetector.cc
        lib/TagFamily.cc
        lib/UnionFindSimple.cc
        )
target_link_libraries(apriltag
        ${EIGEN3_LIBS}
        ${OpenCV_LIBS}
        )