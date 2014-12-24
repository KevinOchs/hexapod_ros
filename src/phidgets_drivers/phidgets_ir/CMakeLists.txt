cmake_minimum_required(VERSION 2.8.3)
project(phidgets_ir)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nodelet phidgets_api roscpp sensor_msgs std_msgs std_srvs tf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_ir
  CATKIN_DEPENDS geometry_msgs nodelet phidgets_api roscpp sensor_msgs std_msgs std_srvs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_ir src/ir_ros_i.cpp)

add_executable(phidgets_ir_node src/phidgets_ir_node.cpp)

add_dependencies(phidgets_ir phidgets_api)

target_link_libraries(phidgets_ir ${catkin_LIBRARIES})
target_link_libraries(phidgets_ir_node ${catkin_LIBRARIES} phidgets_ir)

install(TARGETS phidgets_ir phidgets_ir_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
