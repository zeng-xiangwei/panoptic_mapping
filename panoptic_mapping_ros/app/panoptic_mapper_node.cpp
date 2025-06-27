#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "panoptic_mapping_ros/panoptic_mapper.h"

class PanopticMapperNode : public rclcpp::Node {
 public:
  PanopticMapperNode(const std::string& name) : rclcpp::Node(name) {
    this->declare_parameter<std::string>("config_path", "");
  }
};

int main(int argc, char** argv) {
  // Start Ros.
  rclcpp::init(argc, argv);

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  // Can not use argc argv to parse, because ros2 has other arguments. It's
  // conflict.
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Setup node.
  auto node = std::make_shared<PanopticMapperNode>("panoptic_mapper");
  panoptic_mapping::PanopticMapper mapper(node);

  // Setup spinning.
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), mapper.getConfig().ros_spinner_threads);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
