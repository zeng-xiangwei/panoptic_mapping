cmake_minimum_required(VERSION 3.0.2)
project(panoptic_mapping_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rosgraph_msgs
  panoptic_mapping_msgs
  tf2_ros
  panoptic_mapping
  panoptic_mapping_ros
  voxblox
  voxblox_ros
)
catkin_package()

#############
# Libraries #
#############

add_library(${PROJECT_NAME}
        src/evaluation/map_evaluator.cpp
        src/mesh_saver.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
        ${PROJECT_SOURCE_DIR}/include 
        ${catkin_INCLUDE_DIRS}
        )

###############
# Executables #
###############

add_executable(single_map_evaluation
        app/single_map_evaluation.cpp
        )
target_link_libraries(single_map_evaluation ${PROJECT_NAME})

add_executable(multi_map_evaluation
        app/multi_map_evaluation.cpp
        )
target_link_libraries(multi_map_evaluation ${PROJECT_NAME})

add_executable(mesh_saver
        app/mesh_saver_node.cpp
        )
target_link_libraries(mesh_saver ${PROJECT_NAME})

##########
# Export #
##########

set(${PROJECT_NAME}_TARGETS)
list(APPEND ${PROJECT_NAME}_TARGETS ${PROJECT_NAME})
list(APPEND ${PROJECT_NAME}_TARGETS single_map_evaluation)
list(APPEND ${PROJECT_NAME}_TARGETS multi_map_evaluation)
list(APPEND ${PROJECT_NAME}_TARGETS mesh_saver)
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


