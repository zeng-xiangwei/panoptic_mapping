#ifndef PANOPTIC_MAPPING_ROS_VISUALIZATION_CHANGED_SUBMAP_VISUALIZER_H_
#define PANOPTIC_MAPPING_ROS_VISUALIZATION_CHANGED_SUBMAP_VISUALIZER_H_

#include <panoptic_mapping/map/submap_collection.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"

#ifdef VLN_MSGS_FOUND
#include <vln_msgs/msg/map_update.hpp>
#endif

namespace panoptic_mapping {

/**
 * @brief 获取变化的物体，包括增、删、改，并将变化的物体信息发布出去
 *
 */
class ChangedSubmapVisualizer {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;

    // 表面点变化数目大于该阈值才认为是变化
    int change_point_threshold = 10;

    // 是否仅计算 AABB 包围盒
    bool only_use_aabb = false;
    // 包围盒是否仅与 z 轴对齐
    bool box_only_align_z = true;
    // 包围盒的最小边长
    float min_obb_length = 0.02f;
    std::string obb_frame_id = "world";
    // 新增submap时，是否要求是满足重复检测条件的
    bool use_redetection_for_add = true;

    // 输出的box是否根据空间位置去重
    bool use_space_unique_boxes = false;
    // 最大重叠比例阈值
    float box_overlap_threshold = 0.5f;

    Config() { setConfigName("ChangedSubmapVisualizer"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  ChangedSubmapVisualizer(const Config& config, rclcpp::Node::SharedPtr node);
  virtual ~ChangedSubmapVisualizer() = default;

  /**
   * @brief 获取变化的物体，包括增、删、改，并将变化的物体信息发布出去
   *
   * @param submaps
   */
  void visualizeChangedSubmaps(SubmapCollection* submaps);

 protected:
  void findChangedSubmaps(SubmapCollection& submaps);
  void publishChanges(const SubmapCollection& submaps);
  void publishChangesForVln(const SubmapCollection& submaps);
  void reset();

  // 将submap_infos_中的kDeleted属性的删除
  void update();

 protected:
  /**
   * @brief 记录submap的信息，辅助获取变化的物体
   *
   */
  enum class ChangeType { kUnChanged = 0, kAdded, kDeleted, kChanged };

  struct OrientedBoundingBox {
    // 盒子中心点
    Eigen::Vector3f box_center = Eigen::Vector3f::Zero();
    // 各轴边长
    Eigen::Vector3f extents = Eigen::Vector3f::Zero();
    // 旋转矩阵（由主成分分析得到）
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();

    // 点云质心
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    // 是否是有效
    bool valid = false;
    // OBB、AABB
    std::string box_type = "AABB";
  };

  struct SubmapInfo {
    int id;  // submap uuid
    std::string name;
    int surface_points_size = 0;
    ChangeType change_type = ChangeType::kUnChanged;
    OrientedBoundingBox obb;
    Color color = Color::Gray();
    std::vector<float> embedding_vector;

    // vllm 给出的额外信息
    VllmDescription vllm_descripts;
  };

  /**
   * @brief 计算最小包围框 (OBB)
   *
   * @param points 点云数据
   * @return 返回计算得到的 OBB
   */
  OrientedBoundingBox computeOBB(const std::vector<IsoSurfacePoint>& points,
                                 float voxel_size);
  OrientedBoundingBox computeZAlignedOBB(
      const std::vector<IsoSurfacePoint>& points, float voxel_size);
  OrientedBoundingBox computeStandardOBB(
      const std::vector<IsoSurfacePoint>& points);
  float computeOBBIoU(const OrientedBoundingBox& obb1,
                      const OrientedBoundingBox& obb2);
  std::vector<IsoSurfacePoint> downsamplePointCloud2D(
      const std::vector<IsoSurfacePoint>& points, float voxel_size);
  Eigen::Matrix2f compute2DCloudCovariance(
      const std::vector<IsoSurfacePoint>& points, float voxel_size);

  /**
   * @brief 与已有的 submap 进行比较，判断 box
   * 是否重复，避免同一个物体存在不同大小的 box（存在同一个物体，activate submap
   * 比 persistent submap 小的情况）
   *
   * @param query_submap
   * @return 是否应该保留输入的 submap
   */
  bool deleteRepeatByOBB(const SubmapInfo& query_submap);

  // ROS.
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      obb_publisher_;

#ifdef VLN_MSGS_FOUND
  rclcpp::Publisher<vln_msgs::msg::MapUpdate>::SharedPtr vln_map_update_pub_;
#endif

 private:
  Config config_;

  std::unordered_map<int, SubmapInfo> submap_infos_;
  const SubmapCollection* previous_submaps_ =
      nullptr;  // Only for tracking, not for use!
  
  // 发布 vln 物体变化消息时，需要等待接收着就绪再发布
  bool subscriber_is_active_ = false;
};

}  // namespace panoptic_mapping

#endif