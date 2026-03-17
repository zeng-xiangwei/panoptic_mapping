#include <scene_graph_rviz_plugin/object_description_display.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/property_tree_model.hpp>

#include <Eigen/Geometry>

#include <QObject>
#include <QVariant>
#include <set>

namespace scene_graph_rviz_plugin
{

// ========================================================================
// ObjectProperty implementation
// ========================================================================

ObjectProperty::ObjectProperty(int64_t object_id, const std::string& name, const std::string& desc,
                               rviz_common::properties::Property* parent, const char* changed_slot)
  : rviz_common::properties::Property(QString::fromStdString(name), QVariant(), "", parent, changed_slot)
  , object_id_(object_id)
  , desc_(desc)
{
  // Description is handled by overridden getDescription() method
}

// ========================================================================
// ObjectDescriptionDisplay implementation
// ========================================================================

ObjectDescriptionDisplay::ObjectDescriptionDisplay()
  : objects_property_(nullptr)
  , topic_property_(nullptr)
  , highlight_topic_property_(nullptr)
  , highlighted_object_id_(-1)
{
}

ObjectDescriptionDisplay::~ObjectDescriptionDisplay()
{
}

void ObjectDescriptionDisplay::onInitialize()
{
  // Create topic property for subscription (optional)
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic",
    "/scene_graph/object_descriptions",
    "Topic to subscribe for object descriptions",
    this,
    SLOT(updateSubscription())
  );

  // Create topic property for highlight publisher (optional)
  highlight_topic_property_ = new rviz_common::properties::StringProperty(
    "Highlight Topic",
    "/scene_graph/highlight_object",
    "Topic to publish highlight markers",
    this,
    SLOT(updateHighlightPublisher())
  );
  
  // Get node for subscriptions
  auto node = context_->getRosNodeAbstraction().lock();
  if (!node) {
    RVIZ_COMMON_LOG_ERROR("Failed to get ROS node");
    return;
  }

  // Create subscription for object descriptions topic (array message)
  updateSubscription();

  // Create publisher for highlight markers
  updateHighlightPublisher();

  // Create property tree structure
  // Root property for objects
  objects_property_ = new rviz_common::properties::Property(
    "Objects",
    QVariant(),
    "Scene graph objects with descriptions",
    this
  );
}

void ObjectDescriptionDisplay::updateSubscription()
{
  if (!topic_property_) {
    return;
  }

  std::string topic_name = topic_property_->getString().toStdString();
  if (topic_name.empty()) {
    return;
  }

  // Get node
  auto node = context_->getRosNodeAbstraction().lock();
  if (!node) {
    return;
  }

  // Reset subscription with new topic
  subscription_ = nullptr;
  subscription_ = node->get_raw_node()->create_subscription<semantic_mapping_interfaces::msg::SceneGraphObjectViewArray>(
    topic_name,
    10,
    [this](const semantic_mapping_interfaces::msg::SceneGraphObjectViewArray::ConstSharedPtr msg) {
      processMessage(msg);
    }
  );
}

void ObjectDescriptionDisplay::updateHighlightPublisher()
{
  if (!highlight_topic_property_) {
    return;
  }

  std::string topic_name = highlight_topic_property_->getString().toStdString();
  if (topic_name.empty()) {
    return;
  }

  // Get node
  auto node = context_->getRosNodeAbstraction().lock();
  if (!node) {
    return;
  }

  // Reset publisher with new topic
  highlight_publisher_ = nullptr;
  highlight_publisher_ = node->get_raw_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
    topic_name,
    10
  );
}

void ObjectDescriptionDisplay::onEnable()
{
  // Subscribe is automatically handled by ROS 2 subscription
}

void ObjectDescriptionDisplay::onDisable()
{
  // Clear all data
  objects_data_.clear();
  last_object_ids_.clear();
  updatePropertyTree();
  clearHighlightMarkers();
}

void ObjectDescriptionDisplay::reset()
{
  objects_data_.clear();
  last_object_ids_.clear();
  highlighted_object_id_ = -1;
  updatePropertyTree();
  clearHighlightMarkers();
}

void ObjectDescriptionDisplay::processMessage(const semantic_mapping_interfaces::msg::SceneGraphObjectViewArray::ConstSharedPtr msg)
{
  // Get current object IDs from the message
  std::set<int64_t> current_object_ids;
  
  // Process each object in the array
  for (const auto& obj : msg->objects) {
    int64_t object_id = obj.id;
    current_object_ids.insert(object_id);
    
    // Store/update object data
    ObjectData data;
    data.name = obj.name;
    data.desc = obj.desc;
    data.pose = obj.pose;
    data.dimensions = obj.dimensions;
    
    objects_data_[object_id] = data;
  }
  
  // Remove objects that no longer exist (incremental update)
  // Objects that were in last_object_ids_ but not in current_object_ids
  for (const auto& old_id : last_object_ids_) {
    if (current_object_ids.find(old_id) == current_object_ids.end()) {
      objects_data_.erase(old_id);
      // If the removed object was highlighted, clear the highlight
      if (highlighted_object_id_ == old_id) {
        clearHighlightMarkers();
        highlighted_object_id_ = -1;
      }
    }
  }
  
  // Update last_object_ids_ for next iteration
  last_object_ids_ = current_object_ids;

  // Update property tree
  updatePropertyTree();
}

void ObjectDescriptionDisplay::updatePropertyTree()
{
  if (!objects_property_) {
    return;
  }

  // Step 1: Update existing properties and create new ones
  for (const auto& pair : objects_data_) {
    int64_t object_id = pair.first;
    const ObjectData& data = pair.second;

    auto it = object_properties_.find(object_id);
    if (it != object_properties_.end()) {
      // Property already exists, just update desc
      ObjectProperty* obj_prop = it->second;
      obj_prop->setDesc(data.desc);
    } else {
      // Create display string: "ID: X - Name"
      std::string display_name = "ID: " + std::to_string(object_id) + " - " + data.name;
      
      // Create object property
      ObjectProperty* obj_prop = new ObjectProperty(
        object_id,
        data.name,
        data.desc,
        objects_property_,
        nullptr  // changed_slot
      );
      obj_prop->setName(display_name.c_str());

      // Connect the clicked signal to onObjectSelected slot
      QObject::connect(obj_prop, &ObjectProperty::clicked, 
                      this, &ObjectDescriptionDisplay::onObjectSelected);

      // Create a checkbox property to detect clicks
      // When user clicks on the checkbox, the checked state changes
      auto click_detector = new rviz_common::properties::BoolProperty(
        "Highlight",
        false,
        "Check to highlight this object in 3D view",
        obj_prop,
        nullptr
      );
      
      // Connect the changed signal to trigger selection
      QObject::connect(click_detector, &rviz_common::properties::BoolProperty::changed, 
                      [this, obj_prop, click_detector]() {
                        if (click_detector->getBool()) {
                          // Reset checkbox immediately for next click
                          click_detector->setBool(false);
                          // Emit clicked signal
                          Q_EMIT obj_prop->clicked(obj_prop);
                        }
                      });

      // Store the property
      object_properties_[object_id] = obj_prop;
    }
  }

  // Step 2: Remove properties that no longer exist
  std::vector<int64_t> ids_to_remove;
  for (const auto& pair : object_properties_) {
    int64_t object_id = pair.first;
    if (objects_data_.find(object_id) == objects_data_.end()) {
      ids_to_remove.push_back(object_id);
    }
  }
  for (int64_t id : ids_to_remove) {
    auto it = object_properties_.find(id);
    if (it != object_properties_.end()) {
      ObjectProperty* obj_prop = it->second;
      // Delete the property (parent will be handled automatically)
      delete obj_prop;
      object_properties_.erase(it);
    }
  }

  // Expand the property tree
  if (objects_property_) {
    objects_property_->expand();
  }
}

void ObjectDescriptionDisplay::onObjectSelected(ObjectProperty* prop)
{
  if (!prop) {
    return;
  }

  int64_t object_id = prop->getObjectId();
  
  // Find object data
  auto it = objects_data_.find(object_id);
  if (it != objects_data_.end()) {
    const ObjectData& data = it->second;
    publishHighlightMarker(object_id, data.pose, data.dimensions);
    highlighted_object_id_ = object_id;
  }
}

void ObjectDescriptionDisplay::publishHighlightMarker(int64_t object_id, const geometry_msgs::msg::Pose& pose, 
                                                        const geometry_msgs::msg::Vector3& dimensions)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "highlight";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // Set color to bright red
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // Line width
  marker.scale.x = 0.05;

  // Get position
  float px = pose.position.x;
  float py = pose.position.y;
  float pz = pose.position.z;
  float dx = dimensions.x / 2.0;
  float dy = dimensions.y / 2.0;
  float dz = dimensions.z / 2.0;

  // Create quaternion from pose orientation
  Eigen::Quaterniond q(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z
  );
  // Normalize to ensure valid rotation
  q.normalize();

  // 8 corners of the box in local coordinates (centered at origin)
  std::vector<Eigen::Vector3d> local_corners = {
    Eigen::Vector3d(-dx, -dy, -dz),  // 0: back left bottom
    Eigen::Vector3d( dx, -dy, -dz),  // 1: back right bottom
    Eigen::Vector3d( dx,  dy, -dz),  // 2: front right bottom
    Eigen::Vector3d(-dx,  dy, -dz),  // 3: front left bottom
    Eigen::Vector3d(-dx, -dy,  dz),  // 4: back left top
    Eigen::Vector3d( dx, -dy,  dz),  // 5: back right top
    Eigen::Vector3d( dx,  dy,  dz),  // 6: front right top
    Eigen::Vector3d(-dx,  dy,  dz),  // 7: front left top
  };

  // Rotate and translate corners to world coordinates
  std::vector<Eigen::Vector3d> world_corners;
  for (const auto& corner : local_corners) {
    Eigen::Vector3d rotated = q * corner;  // Rotate
    rotated += Eigen::Vector3d(px, py, pz);  // Translate
    world_corners.push_back(rotated);
  }

  // 12 edges of the box (pairs of corner indices)
  std::vector<std::pair<int, int>> edges = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},  // bottom
    {4, 5}, {5, 6}, {6, 7}, {7, 4},  // top
    {0, 4}, {1, 5}, {2, 6}, {3, 7},  // vertical
  };

  for (const auto& edge : edges) {
    const auto& p1 = world_corners[edge.first];
    const auto& p2 = world_corners[edge.second];

    geometry_msgs::msg::Point pt1;
    pt1.x = p1.x(); pt1.y = p1.y(); pt1.z = p1.z();
    marker.points.push_back(pt1);

    geometry_msgs::msg::Point pt2;
    pt2.x = p2.x(); pt2.y = p2.y(); pt2.z = p2.z();
    marker.points.push_back(pt2);
  }

  marker_array.markers.push_back(marker);
  highlight_publisher_->publish(marker_array);
}

void ObjectDescriptionDisplay::clearHighlightMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "highlight";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;

  marker_array.markers.push_back(marker);
  highlight_publisher_->publish(marker_array);
}

// ========================================================================
// Plugin export
// ========================================================================

}  // namespace scene_graph_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(scene_graph_rviz_plugin::ObjectDescriptionDisplay, rviz_common::Display)
