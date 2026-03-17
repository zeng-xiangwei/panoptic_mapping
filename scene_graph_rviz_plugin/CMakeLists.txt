cmake_minimum_required(VERSION 3.8)
project(scene_graph_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clong")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE Release)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

find_package(semantic_mapping_interfaces REQUIRED)

set(DEPENDENCIES_COMBINE
  rclcpp
  geometry_msgs
  visualization_msgs
  pluginlib
  rviz_common
  rviz_rendering
  semantic_mapping_interfaces
)

qt5_wrap_cpp(MOC_FILES
  include/scene_graph_rviz_plugin/object_description_display.h
)

# Add library
add_library(${PROJECT_NAME} SHARED
  src/object_description_display.cpp
  ${MOC_FILES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES_COMBINE})

##########
# EXPORT #
##########
install(
  DIRECTORY include/
  DESTINATION include)

install(
  FILES plugins_description.xml
  DESTINATION share)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)
ament_package()
