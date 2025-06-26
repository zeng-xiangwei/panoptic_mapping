#ifndef PANOPTIC_MAPPING_ROS_VISUALIZATION_SINGLE_TSDF_VISUALIZER_H_
#define PANOPTIC_MAPPING_ROS_VISUALIZATION_SINGLE_TSDF_VISUALIZER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/common/globals.h>
#include <panoptic_mapping/map/submap_collection.h>

#include "panoptic_mapping_ros/visualization/submap_visualizer.h"

namespace panoptic_mapping {

/**
 * @brief A visualizer tailored to work on the single TSDF emulator. Use
 * together with SingleTsdfTracker and SingleTsdfIntegrator.
 */
class SingleTsdfVisualizer : public SubmapVisualizer {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    // Factor which multiplies the entropy value in order to bring it into
    // [0,1] range for visualization.
    float entropy_factor = 1.0f;
    // Normalisation to bing Score into [0, 1] range
    float min_score = 0.0f;
    float max_score = 1.0f;

    // Standard visualizer config.
    SubmapVisualizer::Config submap_visualizer;

    Config() { setConfigName("SingleTsdfVisualizer"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Constructors.
  SingleTsdfVisualizer(const Config& config, std::shared_ptr<Globals> globals,
                       rclcpp::Node::SharedPtr node, bool print_config = true);
  ~SingleTsdfVisualizer() override = default;

  // Visualization message creation.
  std::vector<voxblox_msgs::msg::MultiMesh> generateMeshMsgs(
      SubmapCollection* submaps) override;

  // Interaction.
  void reset() override;
  void clearMesh() override;
  void setVisualizationMode(VisualizationMode visualization_mode) override;
  void setColorMode(ColorMode color_mode) override;

 protected:
  void colorMeshBlockFromClass(const Submap& submap,
                               voxblox_msgs::msg::MeshBlock* mesh_block);
  void colorMeshBlockFromScore(const Submap& submap,
                               voxblox_msgs::msg::MeshBlock* mesh_block);
  std::function<Color(const ClassVoxel&)> getColoring() const;
  void updateVisInfos(const SubmapCollection& submaps) override;

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<
      SubmapVisualizer, SingleTsdfVisualizer, std::shared_ptr<Globals>,
      rclcpp::Node::SharedPtr>
      registration_;

  // NOTE(schmluk): This namespace could also be unique per visualizer to allow
  // multiple meshes to be shown on the same topic.
  const std::string map_name_space_ = "single_tsdf";

  // Cached / tracked data.
  SubmapVisInfo info_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_VISUALIZATION_SINGLE_TSDF_VISUALIZER_H_
