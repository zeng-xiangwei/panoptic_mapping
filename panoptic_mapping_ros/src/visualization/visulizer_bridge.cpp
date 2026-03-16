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
  detect_input_sub_ =
      node_->create_subscription<voxblox_msgs::msg::MultiMeshList>(
          "visualization/submaps/mesh", 10,
          [this](const voxblox_msgs::msg::MultiMeshList::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            message_queue_.push(msg);
            queue_cv_.notify_one();
          });

  undetect_input_sub_ =
      node_->create_subscription<voxblox_msgs::msg::MultiMeshList>(
          "/single_tsdf_for_undetected/visualization/submaps/mesh", 10,
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
    // 等待消息
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock,
                     [this] { return !message_queue_.empty() || stop_flag_; });

      if (stop_flag_ && message_queue_.empty()) {
        break;
      }

      // 没有订阅者时，不处理消息，等待有订阅者时再批量处理
      if (output_pub_->get_subscription_count() == 0) {
        continue;
      }

      // 批量处理：将队列中的所有消息合并为一个
      if (message_queue_.empty()) {
        continue;
      }

      // 合并队列中的所有消息
      auto merged_msg = mergeMessages();

      // 通过 processMessage 处理合并后的消息
      if (merged_msg && !merged_msg->meshlist.empty()) {
        processMessage(merged_msg);
      }
    }
  }
}

voxblox_msgs::msg::MultiMeshList::SharedPtr VisulizerBridge::mergeMessages() {
  // 用于跟踪每个 namespace 的 block 数据，按 block index 去重保留最新
  // key: namespace, value: unordered_map<block_index, MeshBlock, BlockHash>
  using BlockIndexHashMap =
      voxblox::AnyIndexHashMapType<voxblox_msgs::msg::MeshBlock>::type;
  std::unordered_map<std::string, BlockIndexHashMap> namespace_blocks;
  std::unordered_map<std::string, MeshMsgInfo> submap_meta_infos;

  // 取出所有消息并合并
  auto header = message_queue_.front()->header;
  while (!message_queue_.empty()) {
    auto msg = message_queue_.front();
    message_queue_.pop();

    // 合并每个 namespace 的增量，按 block index 去重保留最新
    for (const auto& mesh_msg : msg->meshlist) {
      const std::string& ns = mesh_msg.name_space;

      MeshMsgInfo meta_info = MeshMsgInfo();
      copyMsgToMetaInfo(mesh_msg, meta_info);
      submap_meta_infos[ns] = meta_info;

      if (mesh_msg.mesh.mesh_blocks.empty()) {
        namespace_blocks[ns] = BlockIndexHashMap();
        continue;
      }

      for (const auto& block : mesh_msg.mesh.mesh_blocks) {
        voxblox::BlockIndex block_idx(block.index[0], block.index[1],
                                      block.index[2]);
        // 按 block index 去重，保留最新的 block 数据
        namespace_blocks[ns][block_idx] = block;
      }
    }
  }

  // 构建合并后的消息
  voxblox_msgs::msg::MultiMeshList::SharedPtr merged_msg =
      std::make_shared<voxblox_msgs::msg::MultiMeshList>();

  for (auto& pair : namespace_blocks) {
    voxblox_msgs::msg::MultiMesh mesh;
    MeshMsgInfo& meta_info = submap_meta_infos[pair.first];
    copyMetaInfoToMsg(meta_info, mesh);

    for (auto& block_pair : pair.second) {
      mesh.mesh.mesh_blocks.push_back(block_pair.second);
    }

    merged_msg->meshlist.push_back(mesh);
  }

  merged_msg->header = header;

  return merged_msg;
}

void VisulizerBridge::copyMetaInfoToMsg(const MeshMsgInfo& meta_info,
                                        voxblox_msgs::msg::MultiMesh& mesh) {
  mesh.alpha = meta_info.alpha;
  mesh.name_space = meta_info.name_space;
  mesh.header = meta_info.header;
  mesh.mesh.header = meta_info.header;
  mesh.mesh.block_edge_length = meta_info.block_edge_length;
}
void VisulizerBridge::copyMsgToMetaInfo(
    const voxblox_msgs::msg::MultiMesh& mesh, MeshMsgInfo& meta_info) {
  meta_info.alpha = mesh.alpha;
  meta_info.name_space = mesh.name_space;
  meta_info.header = mesh.header;
  meta_info.block_edge_length = mesh.mesh.block_edge_length;
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
        voxblox::BlockIndex block_idx(block.index[0], block.index[1],
                                      block.index[2]);
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