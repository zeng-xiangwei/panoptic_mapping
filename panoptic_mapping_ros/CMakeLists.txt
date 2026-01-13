cmake_minimum_required(VERSION 3.5)
project(panoptic_mapping_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(panoptic_mapping REQUIRED)
find_package(panoptic_mapping_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(glog REQUIRED)

find_package(vln_msgs QUIET)

set(DEPENDENCIES_COMBINE
    rclcpp
    builtin_interfaces
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    rosgraph_msgs
    visualization_msgs
    tf2_ros
    pcl_conversions
    cv_bridge
    voxblox
    voxblox_msgs
    panoptic_mapping
    panoptic_mapping_msgs
)

if (vln_msgs_FOUND)
    message(STATUS "Found vln_msgs package")
    list(APPEND DEPENDENCIES_COMBINE vln_msgs)
    add_definitions(-DVLN_MSGS_FOUND)
else()
    message(STATUS "vln_msgs package not found")
endif()


#############
# Libraries #
#############

add_library(${PROJECT_NAME} SHARED
        src/panoptic_mapper.cpp
        src/input/input_synchronizer.cpp
        src/visualization/submap_visualizer.cpp
        src/visualization/single_tsdf_visualizer.cpp
        src/visualization/planning_visualizer.cpp
        src/visualization/tracking_visualizer.cpp
        src/visualization/changed_submap_visualizer.cpp
        src/visualization/visulizer_bridge.cpp
        src/conversions/conversions.cpp
        )

target_link_libraries(${PROJECT_NAME}
    yaml-cpp 
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    stdc++fs
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# ament_target_dependencies could add libraries and includes
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES_COMBINE})

###############
# Executables #
###############

add_executable(panoptic_mapper_node
        app/panoptic_mapper_node.cpp
        )
target_link_libraries(panoptic_mapper_node 
    ${PROJECT_NAME} 
    ${rclcpp_LIBRARIES}
    glog::glog
)
ament_target_dependencies(panoptic_mapper_node ${DEPENDENCIES_COMBINE})

# Visulizer bridge node
add_executable(visulizer_bridge_node
  app/visulizer_bridge_node.cpp
)
target_link_libraries(visulizer_bridge_node
    ${PROJECT_NAME} 
    glog::glog
)
ament_target_dependencies(visulizer_bridge_node
  ${DEPENDENCIES_COMBINE}
)

###########
# Export #
###########
# Library
set(${PROJECT_NAME}_TARGETS)
list(APPEND ${PROJECT_NAME}_TARGETS ${PROJECT_NAME})
install(TARGETS ${${PROJECT_NAME}_TARGETS}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
)

install(TARGETS panoptic_mapper_node visulizer_bridge_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Headers
set(${PROJECT_NAME}_LOCAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(DIRECTORY ${${PROJECT_NAME}_LOCAL_INCLUDE_DIR}/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)

# resources
set(_shared_content_folders config launch rviz urdf meshes maps worlds Media param)
foreach(_folder ${_shared_content_folders})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${_folder})
        message(STATUS "Marking SHARED CONTENT FOLDER \"${_folder}\" of package \"${PROJECT_NAME}\" for installation")
        install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${_folder}/
            DESTINATION share/${PROJECT_NAME}/${_folder}
            )
    endif()
endforeach()

# Dependencies
set(${PROJECT_NAME}_RUN_DEPENDS ${DEPENDENCIES_COMBINE})

set(${PROJECT_NAME}_LIBRARIES)
list(APPEND ${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME})

ament_export_include_directories(${${PROJECT_NAME}_LOCAL_INCLUDE_DIR})
ament_export_libraries(${${PROJECT_NAME}_LIBRARIES})
ament_export_dependencies(${${PROJECT_NAME}_RUN_DEPENDS})

ament_package()

