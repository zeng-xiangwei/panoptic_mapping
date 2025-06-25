#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MULTI_MESH_DISPLAY_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MULTI_MESH_DISPLAY_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <voxblox_msgs/msg/multi_mesh.hpp>

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

namespace voxblox_rviz_plugin {

class VoxbloxMeshVisual;
class VisibilityField;

class VoxbloxMultiMeshDisplay
    : public rviz_common::MessageFilterDisplay<voxblox_msgs::msg::MultiMesh> {
  Q_OBJECT

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxbloxMultiMeshDisplay();
  ~VoxbloxMultiMeshDisplay() override = default;
  void updateVisible();

 protected:
  void reset() override;
  void fixedFrameChanged() override;

  // Automatically update the mesh poses based on their frame_ids.
  void update(float wall_dt, float ros_dt) override;

 private:
  void processMessage(
      const voxblox_msgs::msg::MultiMesh::ConstSharedPtr msg) override;
  bool updateTransformation(VoxbloxMeshVisual* visual, rclcpp::Time stamp);
  void updateAllTransformations();

  // The set of all visuals, identified by namespace.
  std::unordered_map<std::string, VoxbloxMeshVisual> visuals_;

  // The root of the visibility tree.
  std::unique_ptr<VisibilityField> visibility_fields_;
  Q_SLOT void visibleSlot();

  // Property to set visibility for all submaps.
  rviz_common::properties::BoolProperty toggle_visibility_all_property_;
  Q_SLOT void toggleVisibilityAllSLOT();

  // Keep track of the time that elapsed since we last updated the submap poses,
  // such that we can throttle these updates to a reasonable rate.
  float dt_since_last_update_;
};

// Allow the user to show hide sets of submaps based on the name spaces.
class VisibilityField : public rviz_common::properties::BoolProperty {
  Q_OBJECT
 public:
  VisibilityField(const std::string& name,
                  rviz_common::properties::BoolProperty* parent,
                  VoxbloxMultiMeshDisplay* master);
  void addField(const std::string& field_name);
  void removeField(const std::string& field_name);
  bool isEnabled(const std::string& field_name);
  void setEnabledForAll(bool enabled);

 private:
  Q_SLOT void visibleSlot();
  VoxbloxMultiMeshDisplay* master_;
  std::unordered_map<std::string, std::unique_ptr<VisibilityField>> children_;
  bool hasNameSpace(const std::string& name, std::string* ns,
                    std::string* sub_name);
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MULTI_MESH_DISPLAY_H_
