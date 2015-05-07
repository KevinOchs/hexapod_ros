cmake_minimum_required(VERSION 2.8.3)
project(fovis_ros)

find_package(catkin REQUIRED COMPONENTS
	libfovis
	roscpp
	sensor_msgs
	nav_msgs
	message_filters
	image_transport
	cv_bridge
	image_geometry
	tf
	pcl_ros
	std_srvs
	message_generation
	std_msgs)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS signals thread)
include_directories(${Boost_INCLUDE_DIRS} ${PCL_ICNLUDE_DIRS})

add_message_files(DIRECTORY msg FILES FovisInfo.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

add_definitions("-msse3")

include_directories(src ${libfovis_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(visualization src/visualization.cpp)

add_executable(fovis_stereo_odometer src/stereo_odometer.cpp)

add_executable(fovis_mono_depth_odometer src/mono_depth_odometer.cpp)

add_dependencies(fovis_mono_depth_odometer fovis_ros_generate_messages_cpp)
add_dependencies(fovis_stereo_odometer fovis_ros_generate_messages_cpp)
add_dependencies(visualization fovis_ros_generate_messages_cpp)

target_link_libraries(fovis_stereo_odometer ${catkin_LIBRARIES} ${PCL_LIBRARIES} visualization)
target_link_libraries(fovis_mono_depth_odometer ${catkin_LIBRARIES} ${PCL_LIBRARIES} visualization)

