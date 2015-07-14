cmake_minimum_required(VERSION 2.8.3)
project(startup)
find_package(catkin REQUIRED COMPONENTS
  nao_msgs
  roscpp
  std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(startup_node src/startup.cpp)
target_link_libraries(startup_node
  ${catkin_LIBRARIES}
)
