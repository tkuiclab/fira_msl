##############################################################################
# Sections
##############################################################################
find_package(Qt4)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

##############################################################################
# Binaries
##############################################################################

add_library(PF_Lib ${QT_SOURCES} ${QT_MOC_HPP})
target_link_libraries(PF_Lib ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(PF_Lib PF_Lib_generate_messagess)

install(TARGETS PF_Lib
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
