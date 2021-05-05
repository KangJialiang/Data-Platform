CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(BUILD_TYPE Release)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(LibZip REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS_RELEASE} ${SSE_FLAGS} -std=c++11"
)

include_directories(
	${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${LIBZIP_INCLUDE_DIR_ZIP}
        ${LIBZIP_INCLUDE_DIR_ZIPCONF}
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

 add_library(responseCalib src/responseCalib.cpp src/FOVUndistorter.cpp src/PhotometricUndistorter.cpp)
 target_link_libraries(responseCalib PRIVATE Qt5::Widgets ${OpenCV_LIBS} ${LIBZIP_LIBRARY})

 add_library(vignetteCalib src/vignetteCalib.cpp src/FOVUndistorter.cpp src/PhotometricUndistorter.cpp)
 target_link_libraries(vignetteCalib PRIVATE Qt5::Widgets ${OpenCV_LIBS} ${LIBZIP_LIBRARY})

 add_library(findPointsInRange src/findPointsInRange.cpp src/FOVUndistorter.cpp)
 target_link_libraries(findPointsInRange PRIVATE Qt5::Widgets ${OpenCV_LIBS} ${LIBZIP_LIBRARY})
