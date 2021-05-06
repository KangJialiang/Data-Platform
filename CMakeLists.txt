cmake_minimum_required(VERSION 3.5)

project(signals_and_slots LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

set(SOURCES
    src/main.cpp
    src/mainwindow.cpp)
set(HEADERS
    include/mainwindow.h)
set(FORMS
    src/mainwindow.ui)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/modules/mono_dataset_code/include
    ${CMAKE_CURRENT_LIST_DIR}/modules/Camera/include)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/modules/mono_dataset_code)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/modules/Camera)

add_executable(signals_and_slots ${SOURCES} ${HEADERS} ${FORMS})
target_link_libraries(signals_and_slots
    PRIVATE Qt5::Widgets
    ${realsense2_LIBRARY}
    ${OpenCV_LIBS}
    responseCalib
    vignetteCalib
    findPointsInRange
    Camera)
