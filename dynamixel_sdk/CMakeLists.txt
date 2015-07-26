cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_sdk)

find_package( catkin REQUIRED COMPONENTS )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_sdk 
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library( dxl_hal src/dxl_hal.cpp )
add_library( dynamixel src/dynamixel.cpp )
target_link_libraries( dynamixel dxl_hal )


install(TARGETS dxl_hal dynamixel
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
 