cmake_minimum_required(VERSION 3.16)
project(lidar_camera_calibrator)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED
        COMPONENTS
        system
        thread
)
find_package(PCL QUIET REQUIRED)
find_package(GTSAM REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

find_package(VTK REQUIRED)

add_definitions(${PCL_DEFINITIONS})

include_directories(
        src
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        thirdParty
        ${VTK_INCLUDE_DIRS}
)

link_directories(
        ${VTK_LIBRARY_DIRS}
)

#qt5_wrap_cpp( MOC mainwindows.h)
#qt5_wrap_ui( UIC mainwindows.ui)

file(GLOB REMOVE_SRC "src/aruco_test.cpp")
file(GLOB UI_SRC "src/*.cpp")
list(REMOVE_ITEM UI_SRC ${REMOVE_SRC})

set(THIRD_PARTY_LIBS
        Qt5::Core
        Qt5::Widgets
        Qt5::Gui
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${VTK_LIBRARIES}
        gtsam
)

add_executable(lidar_camera_calibrator ${UI_SRC})
target_link_libraries(lidar_camera_calibrator
        ${THIRD_PARTY_LIBS}
)

# add_executable(aruco_test src/aruco_test.cpp src/board_corner_detection.cpp)
# target_link_libraries(aruco_test
#         ${THIRD_PARTY_LIBS})