cmake_minimum_required(VERSION 2.8.3)
project( hexapod_controller )

find_package( catkin REQUIRED COMPONENTS nodelet roscpp geometry_msgs sensor_msgs std_msgs hexapod_msgs tf )

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hexapod_controller 
  CATKIN_DEPENDS nodelet roscpp geometry_msgs sensor_msgs std_msgs hexapod_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library( control src/control.cpp )
add_dependencies( control hexapod_msgs_generate_messages_cpp )

add_library( gait src/gait.cpp )
add_dependencies( gait hexapod_msgs_generate_messages_cpp )

add_library( ik src/ik.cpp )
add_dependencies( ik hexapod_msgs_generate_messages_cpp )

add_library( servo_driver src/servo_driver.cpp )
target_link_libraries( servo_driver ${catkin_LIBRARIES} dynamixel )
add_dependencies( servo_driver hexapod_msgs_generate_messages_cpp )

add_executable( hexapod_controller src/hexapod_controller.cpp )
target_link_libraries( hexapod_controller ${catkin_LIBRARIES} control gait ik servo_driver )
add_dependencies( hexapod_controller hexapod_msgs_generate_messages_cpp )

install(TARGETS hexapod_controller
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

install(FILES hexapod_controller.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

