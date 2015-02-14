cmake_minimum_required(VERSION 2.8.3)
project(sonar_positioning)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	tf
	message_filters
	)

find_package(Boost REQUIRED COMPONENTS system)


## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS 
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	tf
	message_filters
	INCLUDE_DIRS
	include
	 )


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} SYSTEM ${Boost_INCLUDE_DIR})

add_executable(sonar_transformer src/sonar_transformation.cpp )
target_link_libraries(sonar_transformer ${catkin_LIBRARIES})

add_executable(sonar_position src/sonar_position.cpp)
target_link_libraries(sonar_position ${catkin_LIBRARIES})

add_dependencies(sonar_position std_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp micron_driver_generate_messages_cpp)
