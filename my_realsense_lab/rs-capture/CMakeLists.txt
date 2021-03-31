cmake_minimum_required(VERSION 3.1.0)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)


project(RealsenseCapture)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV QUIET)
find_package(realsense2)
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(rs-capture  rs-capture.cpp)
add_executable(showraw  showraw.cpp)
target_link_libraries(rs-capture PRIVATE realsense2::realsense2 ${OpenCV_LIBS})
target_link_libraries(showraw PRIVATE realsense2::realsense2 ${OpenCV_LIBS})