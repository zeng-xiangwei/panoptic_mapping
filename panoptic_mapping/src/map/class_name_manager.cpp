#include "panoptic_mapping/map/class_name_manager.h"

namespace panoptic_mapping {

ClassNameManager* ClassNameManager::getGlobalInstance() {
  static ClassNameManager instance;
  return &instance;
}

int ClassNameManager::getClassID(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = name_to_id_.find(name);
  if (it != name_to_id_.end()) {
    return it->second;
  }
  int new_id = next_id_++;
  name_to_id_[name] = new_id;
  id_to_name_[new_id] = name;
  return new_id;
}

}  // namespace panoptic_mapping