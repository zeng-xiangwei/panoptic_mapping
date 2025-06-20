cmake_minimum_required(VERSION 3.0.2)
project(panoptic_mapping_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  message_generation
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
  rosgraph_msgs
  tf2_ros
  minkindr_conversions
  voxblox
  voxblox_ros
  panoptic_mapping
  panoptic_mapping_msgs
)
catkin_package()

find_package(OpenCV REQUIRED)

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
    ${catkin_LIBRARIES}
    yaml-cpp 
    ${OpenCV_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
)
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
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Headers
set(${PROJECT_NAME}_LOCAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(DIRECTORY ${PROJECT_NAME}_LOCAL_INCLUDE_DIR
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)

# Dependencies
set(${PROJECT_NAME}_RUN_DEPENDS)

set(${PROJECT_NAME}_LIBRARIES)
list(APPEND ${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME})
catkin_package(
    INCLUDE_DIRS ${${PROJECT_NAME}_LOCAL_INCLUDE_DIR}
    LIBRARIES ${${PROJECT_NAME}_LIBRARIES}
    CATKIN_DEPENDS ${${PROJECT_NAME}_RUN_DEPENDS}
)

