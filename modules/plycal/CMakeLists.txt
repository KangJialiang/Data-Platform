set(CMAKE_INCLUDE_CURRENT_DIR ON)
# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI file

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(VTK REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${VTK_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "include/*.h")
file(GLOB FORMS "src/*.ui")

add_library(plycal ${SOURCES} ${HEADERS} ${FORMS})

target_link_libraries(
	plycal
	PRIVATE
	Qt5::Widgets
	${VTK_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${PCL_LIBRARIES}
	${CERES_LIBRARIES})