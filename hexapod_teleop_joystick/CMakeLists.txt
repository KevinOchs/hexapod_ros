cmake_minimum_required(VERSION 2.8.3)
project(hexapod_teleop_joystick)

find_package( catkin REQUIRED COMPONENTS nodelet roscpp sensor_msgs hexapod_msgs )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hexapod_teleop_joystick 
  CATKIN_DEPENDS nodelet roscpp sensor_msgs hexapod_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable( hexapod_teleop_joystick src/hexapod_teleop_joystick.cpp ) 
target_link_libraries( hexapod_teleop_joystick ${catkin_LIBRARIES} )
add_dependencies( hexapod_teleop_joystick hexapod_msgs_generate_messages_cpp )

install(TARGETS hexapod_teleop_joystick
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

install(FILES hexapod_teleop_joystick.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
 