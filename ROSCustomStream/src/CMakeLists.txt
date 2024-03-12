cmake_minimum_required(VERSION 3.15)
project(roscustomstream)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Threads REQUIRED)
find_package(ImFusionLib REQUIRED)

# Your existing find_package calls and other configurations remain the same.

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp

  # DEPENDS other non-ROS dependencies
)

# Define target library



set(RosCustomStreamHeaders
        CustomROSTopicTrackingStream.h
        CustomROSPlugin.h)

set(RosCustomStreamSources
        CustomROSTopicTrackingStream.cpp
        CustomROSPlugin.cpp)

add_library(${PROJECT_NAME} SHARED ${RosCustomStreamSources} ${RosCustomStreamHeaders})



target_include_directories(${PROJECT_NAME} PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/msgs>
        include
        ${IMFUSION_SDK_INCLUDE}
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        )


target_link_libraries(${PROJECT_NAME} PRIVATE
        ImFusionLib
        ImFusionStream
        ImFusionROS
        ImFusionRobotics
        ${catkin_LIBRARIES}
        )

# If you have any executables, use add_executable and target_link_libraries here

install(TARGETS ${PROJECT_NAME}
        EXPORT "${PROJECT_NAME}Targets"
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install other project files like launch files, etc.
# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
