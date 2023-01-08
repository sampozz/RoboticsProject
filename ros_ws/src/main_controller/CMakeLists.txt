cmake_minimum_required(VERSION 3.0.2)
project(main_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ur5_controller
  shelfino_controller
  gazebo_msgs
)

find_package(Eigen3 REQUIRED)

add_compile_options(-std=c++11)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
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
  src/fsm_utils.cpp
  src/fsm_states_ass_1.cpp
  src/fsm_states_ass_2.cpp
  src/fsm_states_ass_3.cpp
  src/fsm_test.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ur5_controller_node.cpp)
add_executable(fsm src/fsm_controller.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(fsm ${catkin_LIBRARIES})
target_link_libraries(fsm main_controller ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
#   scripts/sin_movement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS fsm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)