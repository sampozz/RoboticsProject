cmake_minimum_required(VERSION 3.0.2)
project(robotic_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  message_generation
)

add_compile_options(-std=c++11)

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
)

add_service_files(
  FILES
  Detect.srv
  Stop.srv
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
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(shelfino_yolo_node src/shelfino_yolo_node.cpp)
add_executable(ur5_yolo_node src/ur5_yolo_node.cpp)

target_link_libraries(shelfino_yolo_node ${catkin_LIBRARIES})
target_link_libraries(ur5_yolo_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

## Mark libraries for installation
install(TARGETS shelfino_yolo_node ur5_yolo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

