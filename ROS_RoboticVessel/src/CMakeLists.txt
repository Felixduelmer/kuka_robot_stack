cmake_minimum_required(VERSION 3.3.0)

message("Building ImFusion Robotic Vessel")
project(robotic_vessel)

include(cmake/Macros.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH ${ROS_WORKSPACE_ROOT})
list(APPEND CMAKE_PREFIX_PATH /home/robotics-verse/projects/felix/libtorch)

find_package(ImFusionLib REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        iiwa_msgs
        iiwa_ros
        )
find_package(OpenIGTLink REQUIRED)

find_package(OpenCV 3 REQUIRED) # locate OpenCV in system #JZL
#include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "This is OpenCV INCLUDE DIRS" ${OpenCV_INCLUDE_DIRS})
message(STATUS "This is OpenCV LIBS DIRS" ${OpenCV_LIBS})

find_package(Torch REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        sensor_msgs
        iiwa_msgs
        iiwa_ros
        DEPENDS OpenIGTLink
        CFG_EXTRAS definitions.cmake
)


include("cmake/definitions.cmake")
file(GLOB PLUGIN_HEADERS "include/*.h" "include/robotic_vessel/*.h")

set(PLUGIN_SOURCES
        src/vessel_segmentation_listener.cpp
        src/sweep_rec_and_comp.cpp
        src/RobotControl.cpp
        src/plugin.cpp
        src/factory.cpp
        src/algorithm.cpp
        src/controller.cpp
        src/controller.ui
        src/vessel_segmentation_listener.cpp include/robotic_vessel/vessel_segmentation_listener.h)

add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCES} ${PLUGIN_FORMS} ${PLUGIN_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
        include
        ${IMFUSION_SDK_INCLUDE}
        ${catkin_INCLUDE_DIRS}
        ${OpenIGTLink_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${Qt5Widgets_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME} PRIVATE
        ImFusionLib
        ImFusionStream
        ImFusionUS
        OpenIGTLink
        ${OpenCV_LIBS}
        "${TORCH_LIBRARIES}"
        ${catkin_LIBRARIES}
        Qt5::Widgets
        )

# Define output target directories
#imfusion_set_common_target_properties()

###### INSTALLATION
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
        EXPORT "${PROJECT_NAME}Targets"
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
