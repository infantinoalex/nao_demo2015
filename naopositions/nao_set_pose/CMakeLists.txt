cmake_minimum_required(VERSION 2.8.3)
project(nao_et_pose

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nao_msgs
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  nao_msgs
  #std_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nao_standup
#  CATKIN_DEPENDS nao_msgs roscpp std_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(nao_standup
#   src/${PROJECT_NAME}/nao_standup.cpp
# )

## Declare a cpp executable
add_executable(set_pose src/set_pose.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(nao_standup_node nao_standup_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(set_pose ${catkin_LIBRARIES})


