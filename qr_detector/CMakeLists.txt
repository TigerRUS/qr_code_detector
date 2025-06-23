cmake_minimum_required(VERSION 3.0.2)
project(qr_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  tf
  image_transport
  std_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

# Improved ZBar finding
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

if(ZBAR_FOUND)
    message(STATUS "Found ZBar: ${ZBAR_LIBRARIES}")
    include_directories(${ZBAR_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "ZBar library not found")
endif()

catkin_package(
  INCLUDE_DIRS include ${ZBAR_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/qr_detector_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)