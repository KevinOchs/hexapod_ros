################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_sdk)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS roscpp)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  LIBRARIES dynamixel_sdk
)

################################################################################
# Build
################################################################################
include_directories(
  include/dynamixel_sdk
  ${catkin_INCLUDE_DIRS}
)

if(APPLE)
  add_library(dynamixel_sdk
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_mac.cpp
  )
else()
  add_library(dynamixel_sdk
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_linux.cpp
  )
endif()

add_dependencies(dynamixel_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_sdk ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS dynamixel_sdk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


################################################################################
# Test
################################################################################
