# check http://www.cmake.org/Wiki/CMake_Editors_Support

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_C_FLAGS "-Wall")

project(robotarm)

find_package(PCL 1.4 REQUIRED)

include_directories(. ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Threads REQUIRED)
include_directories(${USB_INCLUDE_DIRS})

add_executable (simple simple.cpp)
add_executable (simplegl simplegl.cpp ../helpers/Mutex.hpp ../helpers/Freenect.hpp)
add_executable (tabledetect tabledetect.cpp ../helpers/Mutex.hpp ../helpers/Freenect.hpp ../helpers/TableTopDetector.cpp ../helpers/TableTopDetector.h ../helpers/CloudUtils.hpp)

# Mac just has everything already
if(APPLE)
	set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")
	
	target_link_libraries (simple freenect ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${PCL_LIBRARIES})
	target_link_libraries (simplegl freenect ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${PCL_LIBRARIES})
	target_link_libraries (tabledetect freenect ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${PCL_LIBRARIES})
	
# Linux, not so much
else()
	find_package(OpenGL REQUIRED)
	find_package(GLUT REQUIRED)

	include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

	target_link_libraries (simple freenect ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${PCL_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
	target_link_libraries (simplegl freenect ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${PCL_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
	target_link_libraries (tabledetect freenect ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${PCL_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
endif()