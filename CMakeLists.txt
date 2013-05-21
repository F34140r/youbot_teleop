cmake_minimum_required(VERSION 2.8.3)
project(youbot_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
########### ROS ###########
find_package(catkin REQUIRED COMPONENTS roscpp joy rosconsole)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)
set(CMAKE_BUILD_TYPE Release)
###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES youbot_teleop
  CATKIN_DEPENDS rosconsole
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

#copy header files
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(PROGRAMS src/youbot_keyboard_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(youbot_joy_teleop src/youbot_joy_teleop.cpp)

target_link_libraries(youbot_joy_teleop ${catkin_LIBRARIES} ${boost_LIBRARIES})
