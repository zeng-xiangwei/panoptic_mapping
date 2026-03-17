#ifndef SCENE_GRAPH_RVIZ_PLUGIN_OBJECT_DESCRIPTION_DISPLAY_H
#define SCENE_GRAPH_RVIZ_PLUGIN_OBJECT_DESCRIPTION_DISPLAY_H

#include <rviz_common/display.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>

#include <QString>

#include <semantic_mapping_interfaces/msg/scene_graph_object_view_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace scene_graph_rviz_plugin
{

// Forward declaration
class ObjectProperty;

/**
 * @brief RViz display plugin for showing scene graph object descriptions in property panel
 */
class ObjectDescriptionDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  ObjectDescriptionDisplay();
  ~ObjectDescriptionDisplay() override;

  // Override rviz::Display methods
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void reset() override;

protected:
  // Topic callback
  void processMessage(const semantic_mapping_interfaces::msg::SceneGraphObjectViewArray::ConstSharedPtr msg);

  // Update subscription when topic property changes
  Q_SLOT void updateSubscription();

  // Update highlight publisher when topic property changes
  Q_SLOT void updateHighlightPublisher();

  // Update property tree with incremental update
  Q_SLOT void updatePropertyTree();

  // Handle object selection
  Q_SLOT void onObjectSelected(ObjectProperty* prop);

  // Publish highlight marker
  void publishHighlightMarker(int64_t object_id, const geometry_msgs::msg::Pose& pose, 
                               const geometry_msgs::msg::Vector3& dimensions);

  // Clear highlight markers
  void clearHighlightMarkers();

private:
  // Subscription for array messages
  rclcpp::Subscription<semantic_mapping_interfaces::msg::SceneGraphObjectViewArray>::SharedPtr subscription_;
  
  // Track previous object IDs for incremental update
  std::set<int64_t> last_object_ids_;

  // Publisher for highlight markers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr highlight_publisher_;

  // Property tree root
  rviz_common::properties::Property* objects_property_;

  // Topic properties (optional)
  rviz_common::properties::StringProperty* topic_property_;
  rviz_common::properties::StringProperty* highlight_topic_property_;

  // Store object data: id -> {name, desc, pose, dimensions}
  struct ObjectData {
    std::string name;
    std::string desc;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Vector3 dimensions;
  };
  std::map<int64_t, ObjectData> objects_data_;

  // Track object properties for incremental update: id -> property
  std::map<int64_t, ObjectProperty*> object_properties_;

  // Currently highlighted object id
  int64_t highlighted_object_id_;
};

/**
 * @brief Property class for individual object display
 */
class ObjectProperty : public rviz_common::properties::Property
{
  Q_OBJECT

public:
  ObjectProperty(int64_t object_id, const std::string& name, const std::string& desc,
                 rviz_common::properties::Property* parent, const char* changed_slot = nullptr);

  int64_t getObjectId() const { return object_id_; }
  // Override getDescription to return QString (required by rviz)
  QString getDescription() const override { return QString::fromStdString(desc_); }
  // Update description
  void setDesc(const std::string& desc) { desc_ = desc; }

Q_SIGNALS:
  // Signal emitted when this property is clicked/activated
  void clicked(ObjectProperty* prop);

private:
  int64_t object_id_;
  std::string desc_;
};

}  // namespace scene_graph_rviz_plugin

#endif  // SCENE_GRAPH_RVIZ_PLUGIN_OBJECT_DESCRIPTION_DISPLAY_H
