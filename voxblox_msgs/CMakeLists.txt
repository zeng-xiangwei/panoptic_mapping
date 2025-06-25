cmake_minimum_required(VERSION 3.5)
project(voxblox_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(msg_files
  msg/Block.msg
  msg/Layer.msg
  msg/LayerWithTrajectory.msg
  msg/Mesh.msg
  msg/MeshBlock.msg
  msg/MultiMesh.msg
  msg/VoxelEvaluationDetails.msg
)

set(srv_files
  srv/FilePath.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs nav_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
