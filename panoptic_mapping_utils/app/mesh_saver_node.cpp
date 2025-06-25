#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "panoptic_mapping_utils/mesh_saver.h"

int main(int argc, char** argv) {
  // Start Ros.
  rclcpp::init(argc, argv);


  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup node.
  auto node = rclcpp::Node::make_shared("mesh_saver");
  panoptic_mapping::MeshSaver mesh_saver(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
