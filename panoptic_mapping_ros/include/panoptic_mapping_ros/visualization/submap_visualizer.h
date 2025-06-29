#ifndef PANOPTIC_MAPPING_ROS_VISUALIZATION_SUBMAP_VISUALIZER_H_
#define PANOPTIC_MAPPING_ROS_VISUALIZATION_SUBMAP_VISUALIZER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/common/globals.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping/tools/coloring.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/msg/multi_mesh.hpp>

#include "panoptic_mapping_ros/conversions/mesh_vis.h"

namespace panoptic_mapping {

class SubmapVisualizer {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    std::string visualization_mode = "all";  // Initial visualization mode.
    std::string color_mode = "color";        // Initial color mode.
    int submap_color_discretization = 20;
    bool visualize_mesh = true;
    bool visualize_tsdf_blocks = true;
    bool visualize_free_space = true;
    bool visualize_bounding_volumes = true;
    bool include_free_space = false;

    // true: other visualization_mode alpha = 0.4; false: alpha = 0
    bool visualize_other_mode = true;
    std::string ros_namespace;

    Config() { setConfigName("SubmapVisualizer"); }

   protected:
    void setupParamsAndPrinting() override;
    void fromRosParam() override;
    void printFields() const override;
    void checkParams() const override;
  };

  // Constructors.
  SubmapVisualizer(const Config& config, std::shared_ptr<Globals> globals,
                   rclcpp::Node::SharedPtr node, bool print_config = true);
  virtual ~SubmapVisualizer() = default;

  // Visualization modes.
  enum class ColorMode {
    kColor = 0,
    kNormals,
    kSubmaps,
    kInstances,
    kClasses,
    kChange,
    kClassification,
    kUncertainty,
    kEntropy,
    kPersistent,
    kScore
  };
  enum class VisualizationMode {
    kAll = 0,
    kActive,
    kActiveOnly,
    kInactive,
    kPersistent
  };

  // Visualization mode conversion.
  static ColorMode colorModeFromString(const std::string& color_mode);
  static std::string colorModeToString(ColorMode color_mode);
  static VisualizationMode visualizationModeFromString(
      const std::string& visualization_mode);
  static std::string visualizationModeToString(
      VisualizationMode visualization_mode);

  // Visualization message creation.
  virtual std::vector<voxblox_msgs::msg::MultiMesh> generateMeshMsgs(
      SubmapCollection* submaps);
  virtual visualization_msgs::msg::MarkerArray generateBlockMsgs(
      const SubmapCollection& submaps);
  virtual pcl::PointCloud<pcl::PointXYZI> generateFreeSpaceMsg(
      const SubmapCollection& submaps);
  virtual visualization_msgs::msg::MarkerArray generateBoundingVolumeMsgs(
      const SubmapCollection& submaps);

  // Publish visualization requests.
  virtual void visualizeAll(SubmapCollection* submaps);
  virtual void visualizeMeshes(SubmapCollection* submaps);
  virtual void visualizeTsdfBlocks(const SubmapCollection& submaps);
  virtual void visualizeFreeSpace(const SubmapCollection& submaps);
  virtual void visualizeBoundingVolume(const SubmapCollection& submaps);
  virtual void publishTfTransforms(const SubmapCollection& submaps);

  // Interaction.
  virtual void reset();
  virtual void clearMesh();
  virtual void setVisualizationMode(VisualizationMode visualization_mode);
  virtual void setColorMode(ColorMode color_mode);
  virtual void setGlobalFrameName(const std::string& frame_name) {
    global_frame_name_ = frame_name;
  }

 protected:
  static const Color kUnknownColor_;

  struct SubmapVisInfo {
    // General.
    int id = 0;  // Corresponding submap id.
    std::string name_space;

    // Visualization data.
    bool republish_everything = false;
    bool was_deleted = false;
    bool change_color = true;
    Color color = kUnknownColor_;
    float alpha = 1.0;
    bool visible = true;

    // Tracking.
    ChangeState previous_change_state;        // kChange
    bool was_active;                          // kActive
    voxblox::BlockIndexList previous_blocks;  // Track deleted blocks.
  };

  virtual void updateVisInfos(const SubmapCollection& submaps);
  virtual void setSubmapVisColor(const Submap& submap, SubmapVisInfo* info);
  virtual void generateClassificationMesh(Submap* submap,
                                          voxblox_msgs::msg::Mesh* mesh);

 protected:
  // Settings.
  VisualizationMode visualization_mode_;
  ColorMode color_mode_;
  std::string global_frame_name_ = "mission";

  // Members.
  std::shared_ptr<Globals> globals_;
  voxblox::ExponentialOffsetIdColorMap id_color_map_;

  // Cached / tracked data.
  std::unordered_map<int, SubmapVisInfo> vis_infos_;
  bool vis_infos_are_updated_ = false;
  const SubmapCollection* previous_submaps_ =
      nullptr;  // Only for tracking, not for use!

  // ROS.
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr freespace_pub_;
  rclcpp::Publisher<voxblox_msgs::msg::MultiMesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      tsdf_blocks_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      bounding_volume_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<
      SubmapVisualizer, SubmapVisualizer, std::shared_ptr<Globals>,
      rclcpp::Node::SharedPtr>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_VISUALIZATION_SUBMAP_VISUALIZER_H_
