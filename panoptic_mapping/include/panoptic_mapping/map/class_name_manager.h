#ifndef PANOPTIC_MAPPING_MAP_CLASS_NAME_MANAGER_H_
#define PANOPTIC_MAPPING_MAP_CLASS_NAME_MANAGER_H_

#include <mutex>
#include <string>
#include <unordered_map>

namespace panoptic_mapping {

class ClassNameManager {
 public:
  static ClassNameManager* getGlobalInstance();

  int getClassID(const std::string& name);

 private:
  ClassNameManager() = default;
  ~ClassNameManager() = default;

  std::unordered_map<std::string, int> name_to_id_;
  std::unordered_map<int, std::string> id_to_name_;
  int next_id_ = 0;
  mutable std::mutex mutex_;

  ClassNameManager(const ClassNameManager&) = delete;
  void operator=(const ClassNameManager&) = delete;
};

}  // namespace panoptic_mapping

#endif