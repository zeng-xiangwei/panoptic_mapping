#include "voxblox_rviz_plugin/material_loader.h"

#include <OgreResourceGroupManager.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace voxblox_rviz_plugin {

bool MaterialLoader::materials_loaded_ = false;

void MaterialLoader::loadMaterials() {
  if (materials_loaded_) {
    return;
  }
  // first instance loads a custom ogre material that supports transparent
  // colors.
  std::string path =
      ament_index_cpp::get_package_share_directory("voxblox_rviz_plugin") +
      "/content/materials";
  Ogre::ResourceGroupManager::getSingletonPtr()->createResourceGroup(
      "VoxbloxMaterials");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      path, "FileSystem", "VoxbloxMaterials", true);
  Ogre::ResourceGroupManager::getSingletonPtr()->initialiseResourceGroup(
      "VoxbloxMaterials");
  Ogre::ResourceGroupManager::getSingletonPtr()->loadResourceGroup(
      "VoxbloxMaterials");
  materials_loaded_ = true;
}

}  // namespace voxblox_rviz_plugin
