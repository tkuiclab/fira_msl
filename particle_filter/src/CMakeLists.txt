##############################################################################
# Sections
##############################################################################

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(particle_filter_node ${QT_SOURCES} ${QT_MOC_HPP})
target_link_libraries(particle_filter_node PF_Lib ${catkin_LIBRARIES})
add_dependencies(particle_filter_node particle_filter_node_gencpp)




































