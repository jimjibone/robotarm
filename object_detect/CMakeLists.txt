# check http://www.cmake.org/Wiki/CMake_Editors_Support

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(armomat)

find_package(PCL 1.1 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OPENCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(common_sources
    aruco/aruco.h
    aruco/board.cpp
    aruco/board.h
    aruco/boarddetector.cpp
    aruco/boarddetector.h
    aruco/cameraparameters.cpp
    aruco/cameraparameters.h
    aruco/cvdrawingutils.cpp
    aruco/cvdrawingutils.h
    aruco/marker.cpp
    aruco/marker.h
    aruco/markerdetector.cpp
    aruco/markerdetector.h

    ply.h
    ply.cpp

    utils.h
    utils.cpp

    table_top_detector.cpp
    table_top_detector.h
)

add_library(htk STATIC ${common_sources})
target_link_libraries(htk ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (table_top_detector_main table_top_detector_main.cpp)
target_link_libraries (table_top_detector_main htk ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (cloud_extrusion cloud_extrusion.cpp)
target_link_libraries (cloud_extrusion ${PCL_LIBRARIES} ${OpenCV_LIBS} htk)

add_executable (marked_viewpoint_modeler marked_viewpoint_modeler.cpp)
target_link_libraries (marked_viewpoint_modeler ${PCL_LIBRARIES} ${OpenCV_LIBS} htk)

add_executable (cluster_recognition cluster_recognition.cpp)
target_link_libraries (cluster_recognition htk ${PCL_LIBRARIES} ${OpenCV_LIBS} flann_cpp)
