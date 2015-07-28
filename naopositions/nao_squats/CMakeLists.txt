cmake_minimum_required(VERSION 2.8.3)
project(nao_squats)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nao_msgs
  roscpp
  std_msgs
  custom_msgs
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


###################################
## catkin specific configuration ##
###################################

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES nao_standup
  #CATKIN_DEPENDS nao_msgs roscpp std_msgs
  #DEPENDS system_lib
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
add_executable(squat src/squat.cpp)
add_executable(squat_copy src/squat_copy.cpp)
add_executable(mid_squat src/mid_squat.cpp)
add_executable(stand src/stand.cpp)
add_executable(fall_back src/fall_backward.cpp)
add_executable(fall_front src/fall_forward.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(squat ${catkin_LIBRARIES})
target_link_libraries(squat_copy ${catkin_LIBRARIES})
target_link_libraries(mid_squat ${catkin_LIBRARIES})
target_link_libraries(stand ${catkin_LIBRARIES})
target_link_libraries(fall_back ${catkin_LIBRARIES})
target_link_libraries(fall_front ${catkin_LIBRARIES})

## Add dependencies 
add_dependencies(squat custom_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(squat_copy custom_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(mid_squat custom_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(stand custom_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(fall_back custom_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(fall_front custom_msgs ${catkin_EXPORTED_TARGETS})
