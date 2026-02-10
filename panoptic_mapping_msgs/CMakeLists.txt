cmake_minimum_required(VERSION 3.5)
project(panoptic_mapping_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/DetectronLabel.msg"
  "msg/DetectronLabels.msg"
  "msg/GetObjectInfoMode.msg"
  "msg/ImageContainer.msg"
  "msg/ObjectGenerate.msg"
  "msg/ObjectRelationship.msg"
  "msg/ObjectSearch.msg"
  "msg/BBox.msg"
)

set(srv_files
  "srv/SaveLoadMap.srv"
  "srv/SetVisualizationMode.srv"
  "srv/RemoveSubmap.srv"
  "srv/SubmapClassNameChange.srv"
  "srv/GetSubmapImageData.srv"
  "srv/GetObjectInfo.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
