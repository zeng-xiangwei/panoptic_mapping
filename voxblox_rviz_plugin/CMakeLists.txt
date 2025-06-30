cmake_minimum_required(VERSION 3.5)
project(voxblox_rviz_plugin)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++17 -Wall)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)

message("rviz_ogre_vendor: ${OGRE_INCLUDE_DIRS}")
message("rviz_ogre_vendor: ${OGRE_LIBRARIES}")
message("rviz_common: ${rviz_common_INCLUDE_DIRS}")
set(DEPENDENCIES_COMBINE
  pluginlib
  rviz_common
  rviz_rendering
  rclcpp
  voxblox
  voxblox_msgs
)

qt5_wrap_cpp(MOC_FILES
  include/voxblox_rviz_plugin/voxblox_mesh_display.h
  include/voxblox_rviz_plugin/voxblox_multi_mesh_display.h
)

set(HEADER_FILES
        include/voxblox_rviz_plugin/voxblox_mesh_visual.h
        include/voxblox_rviz_plugin/material_loader.h
        )

set(SRC_FILES
        src/voxblox_mesh_display.cc
        src/voxblox_multi_mesh_display.cc
        src/voxblox_mesh_visual.cc
        src/material_loader.cc
        )

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${HEADER_FILES}
  ${MOC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# ament_target_dependencies could add libraries and includes
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES_COMBINE})


##########
# EXPORT #
##########
install(
  DIRECTORY include/
  DESTINATION include)

install(
  FILES plugin_description.xml
  DESTINATION share)

install(
  DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
register_rviz_ogre_media_exports(DIRECTORIES "content/materials")
ament_package()