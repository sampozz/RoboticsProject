cmake_minimum_required(VERSION 3.0.2)
project(ur5_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  kinematics_lib
)

find_package(Eigen3 REQUIRED)

add_compile_options(-std=c++11)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package()

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

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ur5_controller_node.cpp)
add_executable(sin src/sin_movement.cpp)
add_executable(joints src/joint_movement.cpp)
add_executable(direct_kin src/direct_kinematics_test.cpp)
add_executable(inverse_kin src/inverse_kinematics_test.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(sin ${catkin_LIBRARIES})
target_link_libraries(joints ${catkin_LIBRARIES})
target_link_libraries(direct_kin ${catkin_LIBRARIES})
target_link_libraries(inverse_kin ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/sin_movement.py
  scripts/homing_procedure.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS sin joints direct_kin inverse_kin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)