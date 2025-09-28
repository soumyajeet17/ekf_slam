cmake_minimum_required(VERSION 3.0.2)
project(my_slam_interfaces)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Ticks.msg
  Landmark.msg
  LandmarkArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# (You don’t need to install anything special for a pure message package)

