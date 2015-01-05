cmake_minimum_required(VERSION 2.8.3)
project( hexapod_locomotion )

find_package( catkin REQUIRED COMPONENTS geometry_msgs nodelet roscpp sensor_msgs std_msgs std_srvs tf hexapod_msgs )

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hexapod_locomotion 
  CATKIN_DEPENDS geometry_msgs nodelet roscpp sensor_msgs std_msgs std_srvs tf hexapod_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library( dxl_hal src/usb2ax/dxl_hal.cpp )
add_library( dynamixel src/usb2ax/dynamixel.cpp )
target_link_libraries( dynamixel dxl_hal )

add_library( control src/control.cpp )

add_library( gait src/gait.cpp )

add_library( ik src/ik.cpp )

add_library( servo_driver src/servo_driver.cpp )
target_link_libraries( servo_driver ${catkin_LIBRARIES} dynamixel )

add_executable( hexapod_locomotion src/locomotion.cpp )
target_link_libraries( hexapod_locomotion ${catkin_LIBRARIES} control gait ik servo_driver )

install(TARGETS hexapod_locomotion
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES locomotion.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

