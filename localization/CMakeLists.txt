cmake_minimum_required(VERSION 2.8.3)
project(localization)

## Find catkin macros and libraries
find_package(catkin)

find_package(catkin REQUIRED COMPONENTS
  qt_build
  roscpp
  std_msgs
  cv_bridge
  message_generation
)

rosbuild_prepare_qt4(QtCore)
################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_service_files(DIRECTORY srv FILES encoder.srv)

generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
  message_runtime
  cv_bridge
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

##############################################################################
# Subprojects
##############################################################################

foreach(subdir
    include
    src
    )
  add_subdirectory(${subdir})
endforeach()

## Declare a cpp library
# add_library(PF_Lib
#   include/particle_filter.cpp
# )

## Declare a cpp executable
# add_executable(particle_filter_node src/particle_filter_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
## add_dependencies(particle_filter_node particle_filter_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(particle_filter_node PF_Lib ${catkin_LIBRARIES})
