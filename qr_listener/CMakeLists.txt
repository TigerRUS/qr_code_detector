cmake_minimum_required(VERSION 2.8.3)
project(qr_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(qr_listener src/qr_listener_node.cpp)
target_link_libraries(qr_listener ${catkin_LIBRARIES})