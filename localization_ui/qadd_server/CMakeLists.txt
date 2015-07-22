##############################################################################
# Sections
##############################################################################

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)
set(QT_FORMS ${CMAKE_CURRENT_SOURCE_DIR}/../ui/qserver_window.ui)
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(qadd_server ${QT_SOURCES} ${QT_MOC_HPP} ${QT_FRRMS_HPP})
target_link_libraries(qadd_server localization_ui)
install(TARGETS qadd_server RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
