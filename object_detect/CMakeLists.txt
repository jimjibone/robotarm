# check http://www.cmake.org/Wiki/CMake_Editors_Support

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_C_FLAGS "-Wall")

project(okpcl)

find_package(PCL 1.4 REQUIRED)
#find_package(OpenCV REQUIRED)

include_directories(. ${PCL_INCLUDE_DIRS})    #${OPENCV_INCLUDE_DIRS}
link_directories(${PCL_LIBRARY_DIRS})       #${OPENCV_LIBRARY_DIRS}
add_definitions(${PCL_DEFINITIONS})

add_executable (okpcl main.cpp)

find_package(Threads REQUIRED)
include_directories(${USB_INCLUDE_DIRS})

target_link_libraries (okpcl freenect ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${PCL_LIBRARIES})     #${OpenCV_LIBS}
