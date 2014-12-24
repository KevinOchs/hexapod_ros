cmake_minimum_required(VERSION 2.8.3)
project(phidgets_imu)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nodelet pluginlib phidgets_api roscpp sensor_msgs std_msgs std_srvs tf)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_imu 
  DEPENDS Boost
  CATKIN_DEPENDS geometry_msgs nodelet pluginlib phidgets_api roscpp sensor_msgs std_msgs std_srvs tf
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(phidgets_imu src/imu_ros_i.cpp)
add_library(phidgets_imu_nodelet src/phidgets_imu_nodelet.cpp)

add_executable(phidgets_imu_node src/phidgets_imu_node.cpp)

add_dependencies(phidgets_imu phidgets_api)

target_link_libraries(phidgets_imu ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(phidgets_imu_nodelet ${catkin_LIBRARIES} phidgets_imu)
target_link_libraries(phidgets_imu_node ${catkin_LIBRARIES} phidgets_imu)

install(TARGETS phidgets_imu phidgets_imu_nodelet phidgets_imu_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES phidgets_imu_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
