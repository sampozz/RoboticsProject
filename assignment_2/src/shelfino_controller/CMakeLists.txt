cmake_minimum_required(VERSION 3.0.2)
project(shelfino_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
  kinematics_lib
)

find_package(Eigen3 REQUIRED)

add_compile_options(-std=c++11)

add_service_files(
  FILES
  MoveTo.srv
  Rotate.srv
  MoveForward.srv
)

add_message_files(
  FILES
  Coordinates.msg
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
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/shelfino_controller_lib.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ur5_controller_node.cpp)
add_executable(shelfino_controller_node src/shelfino_controller_node.cpp)
# add_executable(shelfino_test src/shelfino_test.cpp)
add_executable(shelfino_stop src/shelfino_stop.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(shelfino_controller_node shelfino_controller ${catkin_LIBRARIES})
target_link_libraries(shelfino_controller_node ${catkin_LIBRARIES})
# target_link_libraries(shelfino_test ${catkin_LIBRARIES})
target_link_libraries(shelfino_stop ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)