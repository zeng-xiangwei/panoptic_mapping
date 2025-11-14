#ifndef PANOPTIC_MAPPING_VLLM_DESCRIPTION_H_
#define PANOPTIC_MAPPING_VLLM_DESCRIPTION_H_

#include <string>
#include <vector>
namespace panoptic_mapping {
struct VllmDescription {
  // 物体类别
  std::string class_name;
  // 颜色
  std::string color;
  // 形状
  std::string shape;
  // 其他描述
  std::string other_descs;

  std::string toString() const {
    return class_name + " " + color + " " + shape + " " + other_descs;
  }
};
}  // namespace panoptic_mapping

#endif