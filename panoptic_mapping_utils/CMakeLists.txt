cmake_minimum_required(VERSION 3.5)
project(panoptic_mapping_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(voxblox REQUIRED)
find_package(panoptic_mapping REQUIRED)
find_package(panoptic_mapping_msgs REQUIRED)

set(DEPENDENCIES_COMBINE
  rclcpp
  builtin_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
  rosgraph_msgs
  panoptic_mapping_msgs
  tf2_ros
  panoptic_mapping
  panoptic_mapping_ros
  voxblox
)

#############
# Libraries #
#############

add_library(${PROJECT_NAME}
        src/mesh_saver.cpp
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        ${PROJECT_SOURCE_DIR}/include 
        )
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES_COMBINE})

###############
# Executables #
###############

add_executable(mesh_saver
        app/mesh_saver_node.cpp
        )
target_link_libraries(mesh_saver ${PROJECT_NAME})

##########
# Export #
##########

set(${PROJECT_NAME}_TARGETS)
list(APPEND ${PROJECT_NAME}_TARGETS ${PROJECT_NAME})
list(APPEND ${PROJECT_NAME}_TARGETS mesh_saver)
install(TARGETS ${${PROJECT_NAME}_TARGETS}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Headers
set(${PROJECT_NAME}_LOCAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(DIRECTORY ${PROJECT_NAME}_LOCAL_INCLUDE_DIR
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)

# Dependencies
set(${PROJECT_NAME}_RUN_DEPENDS ${DEPENDENCIES_COMBINE})

set(${PROJECT_NAME}_LIBRARIES)
list(APPEND ${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME})

ament_export_include_directories(${${PROJECT_NAME}_LOCAL_INCLUDE_DIR})
ament_export_libraries(${${PROJECT_NAME}_LIBRARIES})
ament_export_dependencies(${${PROJECT_NAME}_RUN_DEPENDS})

ament_package()


