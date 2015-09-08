# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hello_radar: 1 messages, 1 services")

set(MSG_I_FLAGS "-Ihello_radar:/home/dan/catkin_ws/src/hello_radar/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hello_radar_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dan/catkin_ws/src/hello_radar/msg/Num.msg" NAME_WE)
add_custom_target(_hello_radar_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hello_radar" "/home/dan/catkin_ws/src/hello_radar/msg/Num.msg" ""
)

get_filename_component(_filename "/home/dan/catkin_ws/src/hello_radar/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_hello_radar_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hello_radar" "/home/dan/catkin_ws/src/hello_radar/srv/AddTwoInts.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hello_radar
  "/home/dan/catkin_ws/src/hello_radar/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hello_radar
)

### Generating Services
_generate_srv_cpp(hello_radar
  "/home/dan/catkin_ws/src/hello_radar/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hello_radar
)

### Generating Module File
_generate_module_cpp(hello_radar
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hello_radar
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hello_radar_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hello_radar_generate_messages hello_radar_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dan/catkin_ws/src/hello_radar/msg/Num.msg" NAME_WE)
add_dependencies(hello_radar_generate_messages_cpp _hello_radar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dan/catkin_ws/src/hello_radar/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(hello_radar_generate_messages_cpp _hello_radar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hello_radar_gencpp)
add_dependencies(hello_radar_gencpp hello_radar_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hello_radar_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hello_radar
  "/home/dan/catkin_ws/src/hello_radar/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hello_radar
)

### Generating Services
_generate_srv_lisp(hello_radar
  "/home/dan/catkin_ws/src/hello_radar/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hello_radar
)

### Generating Module File
_generate_module_lisp(hello_radar
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hello_radar
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hello_radar_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hello_radar_generate_messages hello_radar_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dan/catkin_ws/src/hello_radar/msg/Num.msg" NAME_WE)
add_dependencies(hello_radar_generate_messages_lisp _hello_radar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dan/catkin_ws/src/hello_radar/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(hello_radar_generate_messages_lisp _hello_radar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hello_radar_genlisp)
add_dependencies(hello_radar_genlisp hello_radar_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hello_radar_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hello_radar
  "/home/dan/catkin_ws/src/hello_radar/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hello_radar
)

### Generating Services
_generate_srv_py(hello_radar
  "/home/dan/catkin_ws/src/hello_radar/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hello_radar
)

### Generating Module File
_generate_module_py(hello_radar
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hello_radar
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hello_radar_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hello_radar_generate_messages hello_radar_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dan/catkin_ws/src/hello_radar/msg/Num.msg" NAME_WE)
add_dependencies(hello_radar_generate_messages_py _hello_radar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dan/catkin_ws/src/hello_radar/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(hello_radar_generate_messages_py _hello_radar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hello_radar_genpy)
add_dependencies(hello_radar_genpy hello_radar_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hello_radar_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hello_radar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hello_radar
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(hello_radar_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hello_radar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hello_radar
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(hello_radar_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hello_radar)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hello_radar\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hello_radar
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(hello_radar_generate_messages_py std_msgs_generate_messages_py)
