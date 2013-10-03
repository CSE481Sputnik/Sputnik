# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "learning_actionlib: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ilearning_actionlib:/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg;-Iactionlib_msgs:/opt/ros/groovy/share/actionlib_msgs/msg;-Istd_msgs:/opt/ros/groovy/share/std_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(learning_actionlib_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperAction.msg"
  "${MSG_I_FLAGS}"
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionResult.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalStatus.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperGoal.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionFeedback.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperFeedback.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionGoal.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperResult.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_cpp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/actionlib_msgs/msg/GoalStatus.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperResult.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_cpp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_cpp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/actionlib_msgs/msg/GoalStatus.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperFeedback.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_cpp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_cpp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperGoal.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_cpp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_actionlib
)

### Generating Services

### Generating Module File
_generate_module_cpp(learning_actionlib
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_actionlib
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(learning_actionlib_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(learning_actionlib_generate_messages learning_actionlib_generate_messages_cpp)

# target for backward compatibility
add_custom_target(learning_actionlib_gencpp)
add_dependencies(learning_actionlib_gencpp learning_actionlib_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS learning_actionlib_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperAction.msg"
  "${MSG_I_FLAGS}"
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionResult.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalStatus.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperGoal.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionFeedback.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperFeedback.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionGoal.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperResult.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_lisp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/actionlib_msgs/msg/GoalStatus.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperResult.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_lisp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_lisp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/actionlib_msgs/msg/GoalStatus.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperFeedback.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_lisp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_lisp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperGoal.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_actionlib
)
_generate_msg_lisp(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_actionlib
)

### Generating Services

### Generating Module File
_generate_module_lisp(learning_actionlib
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_actionlib
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(learning_actionlib_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(learning_actionlib_generate_messages learning_actionlib_generate_messages_lisp)

# target for backward compatibility
add_custom_target(learning_actionlib_genlisp)
add_dependencies(learning_actionlib_genlisp learning_actionlib_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS learning_actionlib_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperAction.msg"
  "${MSG_I_FLAGS}"
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionResult.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalStatus.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperGoal.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionFeedback.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperFeedback.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionGoal.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperResult.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_actionlib
)
_generate_msg_py(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/actionlib_msgs/msg/GoalStatus.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperResult.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_actionlib
)
_generate_msg_py(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_actionlib
)
_generate_msg_py(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/actionlib_msgs/msg/GoalStatus.msg;/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperFeedback.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_actionlib
)
_generate_msg_py(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_actionlib
)
_generate_msg_py(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/actionlib_msgs/msg/GoalID.msg;/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperGoal.msg;/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_actionlib
)
_generate_msg_py(learning_actionlib
  "/home/motioncap/catkin_ws/devel/share/learning_actionlib/msg/GripperResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_actionlib
)

### Generating Services

### Generating Module File
_generate_module_py(learning_actionlib
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_actionlib
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(learning_actionlib_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(learning_actionlib_generate_messages learning_actionlib_generate_messages_py)

# target for backward compatibility
add_custom_target(learning_actionlib_genpy)
add_dependencies(learning_actionlib_genpy learning_actionlib_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS learning_actionlib_generate_messages_py)


debug_message(2 "learning_actionlib: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/learning_actionlib
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(learning_actionlib_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
add_dependencies(learning_actionlib_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/learning_actionlib
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(learning_actionlib_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
add_dependencies(learning_actionlib_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_actionlib\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/learning_actionlib
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(learning_actionlib_generate_messages_py actionlib_msgs_generate_messages_py)
add_dependencies(learning_actionlib_generate_messages_py std_msgs_generate_messages_py)
