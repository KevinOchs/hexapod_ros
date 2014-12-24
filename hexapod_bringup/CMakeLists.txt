cmake_minimum_required(VERSION 2.8.3)
project(hexapod_bringup)

find_package( catkin REQUIRED COMPONENTS )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hexapod_bringup
  CATKIN_DEPENDS
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES hexapod_bringup.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
 