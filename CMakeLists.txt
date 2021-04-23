cmake_minimum_required(VERSION 3.5)

project(signals_and_slots LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

set(SOURCES
    src/camera.cpp
    src/main.cpp
    src/mainwindow.cpp)
set(HEADERS
    include/camera.h
    include/mainwindow.h)
set(FORMS
    src/mainwindow.ui)

add_executable(signals_and_slots ${SOURCES} ${HEADERS} ${FORMS})

target_link_libraries(signals_and_slots PRIVATE Qt5::Widgets ${realsense2_LIBRARY} ${OpenCV_LIBS})
