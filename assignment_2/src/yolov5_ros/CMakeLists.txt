cmake_minimum_required(VERSION 3.0.2)
project(yolov5_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
)

add_service_files(
  FILES
  Detect.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs
  CATKIN_DEPENDS roscpp
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_node src/vision_node.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

