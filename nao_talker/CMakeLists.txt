cmake_minimum_required(VERSION 2.8.3)
project(nao_talker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  LIBRARIES nao_talker
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nao_talker_node src/nao_talker.cpp)

target_link_libraries(nao_talker_node
  ${catkin_LIBRARIES}
)

