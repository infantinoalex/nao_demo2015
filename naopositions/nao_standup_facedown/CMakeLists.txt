
# Header
cmake_minimum_required(VERSION 2.8.3)
project(nao_standup_facedown)


## Find Catkin Macros and Libraries
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
add_executable(stand_up_fd src/stand_up_fd.cpp)


# Add CMake Target Dependancies of the Executable/Library
add_dependencies(stand_up_fd custom_msgs ${catkin_EXPORTED_TARGETS})


# Specify Libraries to Link a Library or Executable Target Against
target_link_libraries(stand_up_fd ${catkin_LIBRARIES})

