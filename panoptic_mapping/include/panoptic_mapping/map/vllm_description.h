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

enum class RelationshipType {
  // from on to
  ON,
  // from under to
  UNDER,
  // 默认关系
  UNKNOWN
};
inline std::string relationshipTypeToString(const RelationshipType& type) {
  switch (type) {
    case RelationshipType::ON:
      return "On";
    case RelationshipType::UNDER:
      return "Under";
  }
  return "Unknown";
}

inline RelationshipType inverseRelationshipType(const RelationshipType& type) {
  switch (type) {
    case RelationshipType::ON:
      return RelationshipType::UNDER;
    case RelationshipType::UNDER:
      return RelationshipType::ON;
  }
  return RelationshipType::UNKNOWN;
}

inline RelationshipType stringToRelationshipType(const std::string& type_str) {
  if (type_str == "On" || type_str == "on" || type_str == "ON") {
    return RelationshipType::ON;
  } else if (type_str == "Under" || type_str == "under" ||
             type_str == "UNDER") {
    return RelationshipType::UNDER;
  }
  return RelationshipType::UNKNOWN;
}

struct VllmRelationship {
  // from id
  int from_id;
  // 描述 from 关系 to
  RelationshipType relationship;
  // to id
  int to_id;

  std::string toString() const {
    return std::to_string(from_id) + " " +
           relationshipTypeToString(relationship) + " " + std::to_string(to_id);
  }
};
}  // namespace panoptic_mapping

#endif