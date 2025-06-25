#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "panoptic_mapping_ros/panoptic_mapper.h"

int main(int argc, char** argv) {
  // Start Ros.
  rclcpp::init(argc, argv);

  // Always add these arguments for proper logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup node.
  auto node = rclcpp::Node::make_shared("panoptic_mapper");
  panoptic_mapping::PanopticMapper mapper(node);

  // Setup spinning.
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), mapper.getConfig().ros_spinner_threads);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
