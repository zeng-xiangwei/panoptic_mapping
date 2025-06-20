cmake_minimum_required(VERSION 3.0.2)
project(panoptic_mapping_msgs)

set(CMAKE_CXX_STANDARD 17)

set(PACKAGE_DEPENDENCIES
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${PACKAGE_DEPENDENCIES})

add_message_files(
  DIRECTORY msg
  FILES
    DetectronLabel.msg
    DetectronLabels.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    SaveLoadMap.srv
    SetVisualizationMode.srv
)

generate_messages(
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    message_runtime
)
