cmake_minimum_required(VERSION 3.5)
project(panoptic_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_definitions(-Wall)
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(voxblox REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)

message("voxblox include: ${voxblox_INCLUDE_DIRS}")

############
# PROTOBUF #
############

include(${PROJECT_SOURCE_DIR}/cmake/functions.cmake)
set(PROTO_DEFNS
        proto/panoptic_mapping/QuatTransformation.proto
        proto/panoptic_mapping/Submap.proto
        proto/panoptic_mapping/SubmapCollection.proto)
set(BASE_PATH "proto")
PROTOBUF_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

add_library(${PROJECT_NAME}_proto ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

#############
# Libraries #
#############

add_library(${PROJECT_NAME}
        src/common/camera.cpp
        src/common/input_data_user.cpp
        src/map/submap.cpp
        src/map/submap_collection.cpp
        src/map/submap_id.cpp
        src/map/instance_id.cpp
        src/map/submap_bounding_volume.cpp
        src/map/classification/binary_count.cpp
        src/map/classification/moving_binary_count.cpp
        src/map/classification/fixed_count.cpp
        src/map/classification/variable_count.cpp
        src/map/classification/uncertainty.cpp
        src/map/scores/average.cpp
        src/map/scores/latest.cpp
        src/map/class_name_manager.cpp
        src/labels/label_handler_base.cpp
        src/labels/null_label_handler.cpp
        src/labels/csv_label_handler.cpp
        src/labels/range_label_handler.cpp
        src/labels/mini_csv_label_handler.cpp
        src/tracking/tracking_info.cpp
        src/tracking/single_tsdf_tracker.cpp
        src/tracking/ground_truth_id_tracker.cpp
        src/tracking/projective_id_tracker.cpp
        src/tracking/detectron_id_tracker.cpp
        src/submap_allocation/null_submap_allocator.cpp
        src/submap_allocation/semantic_submap_allocator.cpp
        src/submap_allocation/monolithic_freespace_allocator.cpp
        src/integration/projective_tsdf_integrator.cpp
        src/integration/class_projective_tsdf_integrator.cpp
        src/integration/single_tsdf_integrator.cpp
        src/integration/projection_interpolators.cpp
        src/integration/mesh_integrator.cpp
        src/map_management/map_manager.cpp
        src/map_management/null_map_manager.cpp
        src/map_management/activity_manager.cpp
        src/map_management/tsdf_registrator.cpp
        src/map_management/layer_manipulator.cpp
        src/map_management/change_detector.cpp
        src/tools/planning_interface.cpp
        src/tools/map_renderer.cpp
        src/tools/null_data_writer.cpp
        src/tools/log_data_writer.cpp
        src/tools/evaluation_data_writer.cpp
        src/tools/serialization.cpp
        src/tools/coloring.cpp
        src/tools/quat_transformation_protobuf_utils.cpp
        )
target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_proto
    ${voxblox_LIBRARIES}
    ${OpenCV_LIBRARY_DIRS}
    stdc++fs
    yaml-cpp
    glog::glog
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${voxblox_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    $<INSTALL_INTERFACE:include>
)

##########
# Export #
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
message("${PROJECT_NAME}_LOCAL_INCLUDE_DIR: ${${PROJECT_NAME}_LOCAL_INCLUDE_DIR}")
install(DIRECTORY ${${PROJECT_NAME}_LOCAL_INCLUDE_DIR}/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)

# Dependencies
set(${PROJECT_NAME}_RUN_DEPENDS)
list(APPEND ${PROJECT_NAME}_RUN_DEPENDS voxblox Protobuf)

set(${PROJECT_NAME}_LIBRARIES)
list(APPEND ${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_proto)

set(${PROJECT_NAME}_proto_include ${CMAKE_INSTALL_PREFIX}/include)
ament_export_include_directories(${${PROJECT_NAME}_LOCAL_INCLUDE_DIR})
ament_export_include_directories(${${PROJECT_NAME}_proto_include})
ament_export_libraries(${${PROJECT_NAME}_LIBRARIES})
ament_export_dependencies(${${PROJECT_NAME}_RUN_DEPENDS})

ament_package()