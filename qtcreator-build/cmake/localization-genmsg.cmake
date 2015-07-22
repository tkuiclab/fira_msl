# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "localization: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(localization_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(localization
  "/home/iclab/FIRA_ws/src/particle_filter/srv/encoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
)

### Generating Module File
_generate_module_cpp(localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(localization_generate_messages localization_generate_messages_cpp)

# target for backward compatibility
add_custom_target(localization_gencpp)
add_dependencies(localization_gencpp localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(localization
  "/home/iclab/FIRA_ws/src/particle_filter/srv/encoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
)

### Generating Module File
_generate_module_lisp(localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(localization_generate_messages localization_generate_messages_lisp)

# target for backward compatibility
add_custom_target(localization_genlisp)
add_dependencies(localization_genlisp localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(localization
  "/home/iclab/FIRA_ws/src/particle_filter/srv/encoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
)

### Generating Module File
_generate_module_py(localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(localization_generate_messages localization_generate_messages_py)

# target for backward compatibility
add_custom_target(localization_genpy)
add_dependencies(localization_genpy localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(localization_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(localization_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(localization_generate_messages_py std_msgs_generate_messages_py)
