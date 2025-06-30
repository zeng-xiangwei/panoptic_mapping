#include "voxblox_rviz_plugin/voxblox_mesh_display.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <tf2_ros/transform_listener.h>
#include <voxblox_rviz_plugin/material_loader.h>

namespace voxblox_rviz_plugin {

VoxbloxMeshDisplay::VoxbloxMeshDisplay()
    : visible_property_(
          "Visible", true,
          "Show or hide the mesh. If the mesh is hidden but not disabled, it "
          "will persist and is incrementally built in the background.",
          this, SLOT(visibleSLOT())) {
  voxblox_rviz_plugin::MaterialLoader::loadMaterials();
}

void VoxbloxMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void VoxbloxMeshDisplay::visibleSLOT() {
  if (visual_) {
    // Set visibility and update the pose if visibility is turned on.
    visual_->setEnabled(visible_property_.getBool());
    if (visible_property_.getBool()) {
      updateTransformation(context_->getClock()->now());
    }
  }
}

void VoxbloxMeshDisplay::processMessage(
    const voxblox_msgs::msg::Mesh::ConstSharedPtr msg) {
  if (!visual_) {
    visual_.reset(
        new VoxbloxMeshVisual(context_->getSceneManager(), scene_node_));
    visual_->setEnabled(visible_property_.getBool());
  }

  // update the frame, pose and mesh of the visual
  visual_->setFrameId(msg->header.frame_id);
  if (updateTransformation(msg->header.stamp)) {
    visual_->setMessage(msg);
  }
}

bool VoxbloxMeshDisplay::updateTransformation(rclcpp::Time stamp) {
  if (!visual_) {
    // can not get the transform if we don't have a visual
    return false;
  }
  // Look up the transform from tf. If it doesn't work we have to skip.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(visual_->getFrameId(), stamp,
                                                 position, orientation)) {
    RCLCPP_DEBUG(rclcpp::get_logger("voxblox_mesh_display"),
                 "Error transforming from frame '%s' to frame '%s'",
                 visual_->getFrameId().c_str(), qPrintable(fixed_frame_));
    return false;
  }
  visual_->setPose(position, orientation);
  return true;
}

void VoxbloxMeshDisplay::fixedFrameChanged() {
  tf_filter_->setTargetFrame(fixed_frame_.toStdString());
  // update the transformation of the visuals w.r.t fixed frame
  updateTransformation(context_->getClock()->now());
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMeshDisplay,
                       rviz_common::Display)
