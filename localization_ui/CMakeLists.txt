##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(localization_ui)
find_package(catkin)

##############################################################################
# Catkin Configuration
##############################################################################

# These find_package calls setup catkin_INCLUDE_DIRS and ROS_LIBRARIES and 
# all the component variables too (e.g. roscpp_INCLUDE_DIR?)
find_package(catkin REQUIRED COMPONENTS qt_build message_generation std_msgs roscpp)
include_directories(${catkin_INCLUDE_DIRS})

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in manifest.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Services
##############################################################################

add_service_files(DIRECTORY srv FILES TwoInts.srv AddTwoInts.srv Envpoint.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs roscpp)

##############################################################################
# Subprojects
##############################################################################

foreach(subdir
    common
    qadd_client
    )
  #install(DIRECTORY ${subdir} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/qt_tutorials)
  add_subdirectory(${subdir})
endforeach()

##############################################################################
# Optional Installs - these aren't really necessary
##############################################################################

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publish src/publish.cpp)
target_link_libraries(publish ${catkin_LIBRARIES})
add_dependencies(publish qtutorials_generate_messages_cpp)

add_executable(carlocation_client src/carlocation_client.cpp)
target_link_libraries(carlocation_client ${catkin_LIBRARIES})
add_dependencies(carlocation_client qtutorials_gencpp)

add_executable(two_ints_server src/two_ints_server.cpp)
target_link_libraries(two_ints_server ${catkin_LIBRARIES})
add_dependencies(two_ints_server qtutorials_gencpp)






