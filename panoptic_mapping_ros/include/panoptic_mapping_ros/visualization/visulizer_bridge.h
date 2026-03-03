/**
 * @file visulizer_bridge.h
 * @brief 语义地图可视化输出转换工具，桥接多个mesh源并转换为软件端期望的格式
 */

#ifndef PANOPTIC_MAPPING_ROS_VISUALIZATION_VISULIZER_BRIDGE_H_
#define PANOPTIC_MAPPING_ROS_VISUALIZATION_VISULIZER_BRIDGE_H_

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <voxblox/core/block_hash.h>
#include <voxblox_msgs/msg/mesh.hpp>
#include <voxblox_msgs/msg/multi_mesh.hpp>
#include <voxblox_msgs/msg/multi_mesh_list.hpp>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"

namespace panoptic_mapping {

/**
 * @brief 语义地图可视化输出转换工具，桥接多个mesh源并转换为软件端期望的格式
 */
class VisulizerBridge {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;

    Config() { setConfigName("VisulizerBridge"); }

    void setupParamsAndPrinting() override;
  };

  struct MeshMsgInfo {
    std_msgs::msg::Header header;
    std::string name_space;
    uint8_t alpha;
    float block_edge_length;
  };

  /**
   * @brief 构造函数
   * @param config 配置参数
   * @param node ROS节点指针
   * @param print_config 是否打印配置信息
   */
  explicit VisulizerBridge(const Config& config, rclcpp::Node::SharedPtr node,
                           bool print_config = true);

  virtual ~VisulizerBridge();

  // 禁止拷贝
  VisulizerBridge(const VisulizerBridge&) = delete;
  VisulizerBridge& operator=(const VisulizerBridge&) = delete;

  /**
   * @brief 重置内部状态
   */
  void reset();

 private:
  // 配置参数
  const Config config_;

  // ROS节点
  rclcpp::Node::SharedPtr node_;

  // 订阅者 - 接收mesh消息
  rclcpp::Subscription<voxblox_msgs::msg::MultiMeshList>::SharedPtr
      detect_input_sub_;

  // 发布者 - 发布转换后的mesh消息
  rclcpp::Publisher<voxblox_msgs::msg::MultiMeshList>::SharedPtr output_pub_;

  // 消息队列相关
  std::queue<voxblox_msgs::msg::MultiMeshList::SharedPtr> message_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  // 消费者线程
  std::thread consumer_thread_;
  std::atomic<bool> stop_flag_{false};

  // 存储已知的block indices，用于删除操作
  std::unordered_map<std::string, voxblox::IndexSet> stored_block_indices_;

  /**
   * @brief 消费队列中的消息
   */
  void consumeMessages();

  /**
   * @brief 合并队列中的所有消息，按 block index 去重保留最新
   * @return 合并后的消息
   */
  voxblox_msgs::msg::MultiMeshList::SharedPtr mergeMessages();

  /**
   * @brief 处理单个消息
   */
  void processMessage(const voxblox_msgs::msg::MultiMeshList::SharedPtr msg);

  void copyMetaInfoToMsg(const MeshMsgInfo& meta_info,
                         voxblox_msgs::msg::MultiMesh& mesh);
  void copyMsgToMetaInfo(const voxblox_msgs::msg::MultiMesh& mesh,
                         MeshMsgInfo& meta_info);
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_VISUALIZATION_VISULIZER_BRIDGE_H_