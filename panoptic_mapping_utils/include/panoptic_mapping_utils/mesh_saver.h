#ifndef PANOPTIC_MAPPING_UTILS_SCANNET_MESHSAVER_H_
#define PANOPTIC_MAPPING_UTILS_SCANNET_MESHSAVER_H_

#include <string>
#include <vector>

#include <panoptic_mapping_ros/conversions/mesh_vis.h>
#include <rclcpp/rclcpp.hpp>
#include <voxblox_msgs/msg/multi_mesh.hpp>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/io/mesh_ply.h"
#include "voxblox/mesh/mesh.h"
#include "voxblox/mesh/mesh_utils.h"

namespace panoptic_mapping {

class MeshSaver {
 public:
  MeshSaver(rclcpp::Node::SharedPtr node);
  virtual ~MeshSaver() = default;
  void gotMeshCallback(const voxblox_msgs::msg::MultiMesh& msg);

 private:
  void setupRos();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<voxblox_msgs::msg::MultiMesh>::SharedPtr mesh_sub_;
};

}  // namespace panoptic_mapping

#endif
