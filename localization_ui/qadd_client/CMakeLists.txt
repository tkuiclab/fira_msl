##############################################################################
# Sections
##############################################################################

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)
set(QT_FORMS ${CMAKE_CURRENT_SOURCE_DIR}/../ui/qclinet_window.ui)
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(qadd_client ${QT_SOURCES} ${QT_MOC_HPP} ${QT_FORMS_HPP})
target_link_libraries(qadd_client localization_ui)
install(TARGETS qadd_client RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
