/**
 * @file visulizer_bridge_node.cpp
 * @brief 使用VisulizerBridge的ROS2节点，用于进行消息合并
 */

#include "panoptic_mapping_ros/visualization/visulizer_bridge.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  // Can not use argc argv to parse, because ros2 has other arguments. It's
  // conflict.
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Setup node.
  auto node = rclcpp::Node::make_shared("visulizer_bridge_node");
  std::string log_dir;
  node->declare_parameter<std::string>("log_dir", "");
  node->get_parameter("log_dir", log_dir);
  if (!log_dir.empty()) {
    FLAGS_log_dir = log_dir;
    LOG(INFO) << "Logging to " << log_dir;
  }
  
  // 创建配置
  panoptic_mapping::VisulizerBridge::Config config;  
  // 创建VisulizerBridge实例
  auto bridge = std::make_shared<panoptic_mapping::VisulizerBridge>(
      config, node, true);
  
  // 运行节点
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}