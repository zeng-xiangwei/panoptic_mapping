function(PROTOBUF_CATKIN_GENERATE_CPP2 BASE_PATH SRCS HDRS)
  if(NOT ARGN)
    message(SEND_ERROR "Error: PROTOBUF_CATKIN_GENERATE_CPP2() called without any proto files")
    return()
  endif(NOT ARGN)

  list(APPEND _protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR}/${BASE_PATH})

  set(${SRCS})
  set(${HDRS})

  # Folder where the proto sources are generated in. This should resolve to
  # "build/project_name/compiled_proto".
  set(COMPILED_PROTO_FOLDER "${CMAKE_CURRENT_BINARY_DIR}/compiled_proto")
  file(MAKE_DIRECTORY ${COMPILED_PROTO_FOLDER})

  # Folder where the generated headers are installed to. This should resolve to
  # "devel/include".
  set(PROTO_GENERATED_HEADERS_INSTALL_DIR
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
  file(MAKE_DIRECTORY ${PROTO_GENERATED_HEADERS_INSTALL_DIR})

  # Folder where the proto files are placed, so that they can be used in other
  # proto definitions. This should resolve to "devel/share/proto".
  set(PROTO_SHARE_SUB_FOLDER "proto")
  set(PROTO_FILE_INSTALL_DIR
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROTO_SHARE_SUB_FOLDER})
  file(MAKE_DIRECTORY ${PROTO_FILE_INSTALL_DIR})

  set(include_candidates
       "/usr/include"
       "${PROTO_FILE_INSTALL_DIR}"
       "${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROTO_SHARE_SUB_FOLDER}"
       "/opt/ros/$ENV{ROS_DISTRO}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROTO_SHARE_SUB_FOLDER}"
       "/opt/ros/$ENV{ROS_DISTRO}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/grpc/"
  )
  foreach(include_candidate ${include_candidates})
    if(EXISTS "${include_candidate}")
      list(APPEND _protobuf_include_path "-I" ${include_candidate})
    endif()
  endforeach()

  _find_protobuf_compiler()
  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    get_filename_component(FIL_REL_DIR ${FIL} DIRECTORY)

    # FIL_REL_DIR contains the path to the proto file, starting inside
    # BASE_PATH. E.g., for base path "proto" and file
    # "proto/project/definitions.proto", FIL_REL_DIR would be "project".
    string(REGEX REPLACE "${BASE_PATH}/?" "" FIL_REL_DIR ${FIL_REL_DIR})

    # Variable for the protobuf share folder (e.g., ../devel/share/proto/project-name)
    set(PROTO_SHARE_FOLDER ${PROTO_FILE_INSTALL_DIR}/${FIL_REL_DIR})
    file(MAKE_DIRECTORY ${PROTO_SHARE_FOLDER})

    # The protoc output folder for C++ .h/.cc files.
    set(OUTPUT_FOLDER ${COMPILED_PROTO_FOLDER}/${FIL_REL_DIR})
    set(OUTPUT_FOLDER_BASE ${COMPILED_PROTO_FOLDER})
    file(MAKE_DIRECTORY ${OUTPUT_FOLDER})
    file(MAKE_DIRECTORY "${PROTO_GENERATED_HEADERS_INSTALL_DIR}/${FIL_REL_DIR}")
    set(OUTPUT_BASE_FILE_NAME "${OUTPUT_FOLDER}/${FIL_WE}")

    list(APPEND ${SRCS} "${OUTPUT_BASE_FILE_NAME}.pb.cc")
    list(APPEND ${HDRS} "${OUTPUT_BASE_FILE_NAME}.pb.h")

    add_custom_command(
      OUTPUT "${OUTPUT_BASE_FILE_NAME}.pb.cc"
             "${OUTPUT_BASE_FILE_NAME}.pb.h"
             "${PROTO_GENERATED_HEADERS_INSTALL_DIR}/${FIL_WE}.pb.h"
             "${PROTO_SHARE_FOLDER}/${FIL_WE}.proto"
      COMMAND  "${PROTOBUF_COMPILER}"
      ARGS --cpp_out ${OUTPUT_FOLDER_BASE} ${_protobuf_include_path} ${ABS_FIL}
      COMMAND ${CMAKE_COMMAND} -E copy
              "${OUTPUT_BASE_FILE_NAME}.pb.h"
              "${PROTO_GENERATED_HEADERS_INSTALL_DIR}/${FIL_REL_DIR}/${FIL_WE}.pb.h"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              ${ABS_FIL}
              "${PROTO_SHARE_FOLDER}/${FIL_WE}.proto"
      DEPENDS ${ABS_FIL}
      COMMENT "Running C++ protocol buffer compiler on ${FIL}."
      VERBATIM)
      install(
          FILES ${ABS_FIL}
          DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROTO_SHARE_SUB_FOLDER}/${FIL_REL_DIR}
      )
  endforeach()

  set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)

  include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
  install(FILES ${${HDRS}}
          DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${FIL_REL_DIR})

endfunction()