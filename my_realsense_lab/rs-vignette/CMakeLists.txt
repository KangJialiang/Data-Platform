# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

project(RealsenseVignetteCalib)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV QUIET)
find_package(realsense2)
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(rs-vignette-collecter  rs-vignette.cpp)
target_link_libraries(rs-vignette-collecter PRIVATE realsense2::realsense2 ${OpenCV_LIBS})


