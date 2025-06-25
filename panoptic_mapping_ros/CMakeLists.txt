cmake_minimum_required(VERSION 3.5)
project(panoptic_mapping_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(voxblox REQUIRED)
find_package(panoptic_mapping REQUIRED)
find_package(panoptic_mapping_msgs REQUIRED)

find_package(OpenCV REQUIRED)

set(DEPENDENCIES_COMBINE
    rclcpp
    builtin_interfaces
    std_msgs
    sensor_msgs
    geometry_msgs
    rosgraph_msgs
    tf2_ros
    cv_bridge
    voxblox
    panoptic_mapping
    panoptic_mapping_msgs
)


#############
# Libraries #
#############

add_library(${PROJECT_NAME}
        src/panoptic_mapper.cpp
        src/input/input_synchronizer.cpp
        src/visualization/submap_visualizer.cpp
        src/visualization/single_tsdf_visualizer.cpp
        src/visualization/planning_visualizer.cpp
        src/visualization/tracking_visualizer.cpp
        src/conversions/conversions.cpp
        )

target_link_libraries(${PROJECT_NAME}
    yaml-cpp 
    ${OpenCV_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# ament_target_dependencies could add libraries and includes
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES_COMBINE})

###############
# Executables #
###############

add_executable(panoptic_mapper_node
        app/panoptic_mapper_node.cpp
        )
target_link_libraries(panoptic_mapper_node ${PROJECT_NAME})

###########
# Export #
###########
# Library
set(${PROJECT_NAME}_TARGETS)
list(APPEND ${PROJECT_NAME}_TARGETS ${PROJECT_NAME})
list(APPEND ${PROJECT_NAME}_TARGETS panoptic_mapper_node)
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

