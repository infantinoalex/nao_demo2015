
# Header 
cmake_minimum_required(VERSION 2.8.3)
project(nao_set_pose)


# Find Catkin Macros and Libraries 
find_package(catkin REQUIRED COMPONENTS
	nao_msgs
	roscpp
	std_msgs
	custom_msgs
)


# Catkin Specific Configuration 
catkin_package(
	#INCLUDE_DIRS include
	#LIBRARIES nao_standup
	#CATKIN_DEPENDS nao_msgs roscpp std_msgs
	#DEPENDS system_lib
)


# Specify Additional Locations of Header Files 
include_directories(
	${catkin_INCLUDE_DIRS}
)


# Declare a CPP Executable 
add_executable(set_pose src/set_pose.cpp)
add_executable(echo src/print_states.cpp)
add_executable(startup src/startup.cpp)


# Add CMake Target Dependancies of the Executable/Library 
add_dependencies(set_pose custom_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(echo custom_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(startup custom_msgs ${catkin_EXPORTED_TARGETS})


# Specify Libraries to Link a Library of Executable Target Against 
target_link_libraries(set_pose ${catkin_LIBRARIES})
target_link_libraries(echo ${catkin_LIBRARIES})
target_link_libraries(startup ${catkin_LIBRARIES})

