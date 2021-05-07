CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS_RELEASE} ${SSE_FLAGS} -std=c++11"
)

include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(Camera src/RsCamera.cpp)
target_link_libraries(Camera ${OpenCV_LIBS})