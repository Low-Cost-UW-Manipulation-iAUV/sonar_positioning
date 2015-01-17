cmake_minimum_required(VERSION 2.8.3)
project(sonar_positioning)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
	tf
	message_generation
	)



## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS 
	roscpp
	std_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
	tf	 
	INCLUDE_DIRS
	include
	 )

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sonar_position src/sonar_position.cpp)
target_link_libraries(sonar_position ${catkin_LIBRARIES})
add_dependencies(sonar_position std_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
