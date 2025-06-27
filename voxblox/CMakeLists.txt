cmake_minimum_required(VERSION 3.5)
project(voxblox)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++14 -Wall -Wextra)

find_package(ament_cmake REQUIRED)

find_package(minkindr REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)

############
# PROTOBUF #
############
include(${PROJECT_SOURCE_DIR}/cmake/functions.cmake)
set(PROTO_DEFNS proto/voxblox/Block.proto
                proto/voxblox/Layer.proto)
set(BASE_PATH "proto")
set(PROTOBUF_COMPILE_PYTHON true)

PROTOBUF_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

####################
# SET SOURCE FILES #
####################

set("${PROJECT_NAME}_SRCS"
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_proto
  ${PROTO_SRCS}
)
target_include_directories(${PROJECT_NAME}_proto SYSTEM PUBLIC
  ${PROTOBUF_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_proto 
  ${PROTOBUF_LIBRARIES}
)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
)
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${EIGEN3_INCLUDE_DIR}
  ${minkindr_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} 
  ${PROJECT_NAME}_proto
  ${minkindr_LIBRARIES}
  glog::glog
)

############
# BINARIES #
############

add_executable(tsdf_to_esdf
  test/tsdf_to_esdf.cc
)
target_link_libraries(tsdf_to_esdf ${PROJECT_NAME})

add_executable(test_load_esdf
  test/test_load_esdf.cc
)
target_link_libraries(test_load_esdf ${PROJECT_NAME})

##########
# EXPORT #
##########
# Library
set(${PROJECT_NAME}_TARGETS)
list(APPEND ${PROJECT_NAME}_TARGETS ${PROJECT_NAME})
list(APPEND ${PROJECT_NAME}_TARGETS ${PROJECT_NAME}_proto)
install(TARGETS ${${PROJECT_NAME}_TARGETS}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Headers
set(${PROJECT_NAME}_LOCAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(DIRECTORY ${${PROJECT_NAME}_LOCAL_INCLUDE_DIR}/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)

# Dependencies
set(${PROJECT_NAME}_RUN_DEPENDS)
list(APPEND ${PROJECT_NAME}_RUN_DEPENDS minkindr Protobuf)

set(${PROJECT_NAME}_LIBRARIES)
list(APPEND ${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_proto)

set(${PROJECT_NAME}_proto_include ${CMAKE_INSTALL_PREFIX}/include)
ament_export_include_directories(${${PROJECT_NAME}_LOCAL_INCLUDE_DIR})
ament_export_include_directories(${${PROJECT_NAME}_proto_include})
ament_export_libraries(${${PROJECT_NAME}_LIBRARIES})
ament_export_dependencies(${${PROJECT_NAME}_RUN_DEPENDS})

ament_package()