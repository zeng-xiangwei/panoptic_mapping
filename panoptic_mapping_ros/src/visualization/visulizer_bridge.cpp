/**
 * @brief
 * 用于与软件端进行数据交互，接收语义建图的话题数据，处理为软件端需要的形式，然后发布
 */

#include "panoptic_mapping_ros/visualization/visulizer_bridge.h"

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include <glog/logging.h>

namespace panoptic_mapping {
void VisulizerBridge::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
}

VisulizerBridge::VisulizerBridge(const Config& config,
                                 rclcpp::Node::SharedPtr node,
                                 bool print_config)
    : config_(config.checkValid()), node_(node) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();

  // 初始化订阅者
  detect_input_sub_ = node_->create_subscription<voxblox_msgs::msg::MultiMeshList>(
      "visualization/submaps/mesh", 10,
      [this](const voxblox_msgs::msg::MultiMeshList::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        message_queue_.push(msg);
        queue_cv_.notify_one();
      });

  // 初始化发布者
  output_pub_ = node_->create_publisher<voxblox_msgs::msg::MultiMeshList>(
      "visualization/converted_mesh", 10);

  // 启动消费者线程
  consumer_thread_ = std::thread(&VisulizerBridge::consumeMessages, this);
}

void VisulizerBridge::consumeMessages() {
  while (rclcpp::ok() && !stop_flag_) {
    voxblox_msgs::msg::MultiMeshList::SharedPtr msg;

    // 等待消息
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock,
                     [this] { return !message_queue_.empty() || stop_flag_; });

      if (stop_flag_ && message_queue_.empty()) {
        break;
      }

      if (!message_queue_.empty()) {
        msg = message_queue_.front();
        message_queue_.pop();
      }
    }

    if (msg) {
      processMessage(msg);
    }
  }
}

void VisulizerBridge::processMessage(
    const voxblox_msgs::msg::MultiMeshList::SharedPtr msg) {
  voxblox_msgs::msg::MultiMeshList processed_msg = *msg;

  // 处理每个mesh
  for (auto& mesh_msg : processed_msg.meshlist) {
    if (mesh_msg.mesh.mesh_blocks.empty()) {
      // 如果mesh block是空的，则将之前存储的block的坐标点设为空（删除操作）
      auto it = stored_block_indices_.find(mesh_msg.name_space);
      if (it != stored_block_indices_.end()) {
        // 创建新的mesh blocks，仅包含之前存储的block indices，但坐标点为空
        for (const auto& block_index : it->second) {
          voxblox_msgs::msg::MeshBlock mesh_block;
          // 设置block索引
          mesh_block.index[0] = block_index.x();
          mesh_block.index[1] = block_index.y();
          mesh_block.index[2] = block_index.z();
          // 保持坐标点为空
          mesh_msg.mesh.mesh_blocks.push_back(mesh_block);
        }

        stored_block_indices_.erase(it);

        LOG(INFO) << "Deleted mesh " << mesh_msg.name_space << " from map.";
      }
    } else {
      // 提取并存储block indices
      for (const auto& block : mesh_msg.mesh.mesh_blocks) {
        voxblox::BlockIndex block_idx(block.index[0], block.index[1], block.index[2]);
        stored_block_indices_[mesh_msg.name_space].insert(block_idx);
      }
    }
  }

  // 发布处理后的消息
  output_pub_->publish(processed_msg);
  static int count = 0;
  LOG(INFO) << "pubed " << count << "mgs";
  count++;
}

void VisulizerBridge::reset() {
  std::lock_guard<std::mutex> lock(queue_mutex_);

  // 清空队列
  while (!message_queue_.empty()) {
    message_queue_.pop();
  }

  // 清空存储的block indices
  stored_block_indices_.clear();
}

VisulizerBridge::~VisulizerBridge() {
  stop_flag_ = true;
  queue_cv_.notify_all();

  if (consumer_thread_.joinable()) {
    consumer_thread_.join();
  }
}

}  // namespace panoptic_mapping