#ifndef PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_
#define PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_

#include <algorithm>
#include <memory>
#include <vector>

#include <panoptic_mapping/common/input_data.h>
#include <panoptic_mapping_msgs/msg/detectron_label.hpp>
#include <panoptic_mapping_msgs/msg/detectron_labels.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/mesh/mesh.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/msg/layer.hpp>

namespace panoptic_mapping {

DetectronLabel detectronLabelFromMsg(
    const panoptic_mapping_msgs::msg::DetectronLabel& msg);

DetectronLabels detectronLabelsFromMsg(
    const panoptic_mapping_msgs::msg::DetectronLabels& msg);

void convertToPointCloud2(
    const Pointcloud& points,
    const voxblox::AlignedVector<Eigen::Matrix<uint8_t, 3, 1>>& colors,
    sensor_msgs::msg::PointCloud2& cloud_msg);

}  // namespace panoptic_mapping

namespace voxblox {

enum class MapDerializationAction : uint8_t {
  kUpdate = 0u,
  kMerge = 1u,
  kReset = 2u
};

inline void colorVoxbloxToMsg(const Color& color,
                              std_msgs::ColorRGBA* color_msg) {
  CHECK_NOTNULL(color_msg);
  color_msg->r = color.r / 255.0;
  color_msg->g = color.g / 255.0;
  color_msg->b = color.b / 255.0;
  color_msg->a = color.a / 255.0;
}

inline void colorMsgToVoxblox(const std_msgs::ColorRGBA& color_msg,
                              Color* color) {
  CHECK_NOTNULL(color);
  color->r = static_cast<uint8_t>(color_msg.r * 255.0);
  color->g = static_cast<uint8_t>(color_msg.g * 255.0);
  color->b = static_cast<uint8_t>(color_msg.b * 255.0);
  color->a = static_cast<uint8_t>(color_msg.a * 255.0);
}

}  // namespace voxblox

#endif  // PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_
