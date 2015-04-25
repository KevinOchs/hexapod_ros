cmake_minimum_required(VERSION 2.8.3)
project(hexapod_sound)

find_package( catkin REQUIRED COMPONENTS nodelet roscpp sound_play hexapod_msgs )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hexapod_sound 
  CATKIN_DEPENDS nodelet roscpp sound_play hexapod_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable( hexapod_sound src/hexapod_sound.cpp ) 
target_link_libraries( hexapod_sound ${catkin_LIBRARIES} )
add_dependencies( hexapod_sound hexapod_msgs_generate_messages_cpp )

install(TARGETS hexapod_sound
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

install(DIRECTORY sounds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sounds
)
install(FILES hexapod_sound.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
 