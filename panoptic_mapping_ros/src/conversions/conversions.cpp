#include "panoptic_mapping_ros/conversions/conversions.h"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace panoptic_mapping {

DetectronLabel detectronLabelFromMsg(
    const panoptic_mapping_msgs::msg::DetectronLabel& msg) {
  DetectronLabel result;
  result.id = msg.id;
  result.instance_id = msg.instance_id;
  result.is_thing = msg.is_thing;
  result.category_id = msg.category_id;
  result.category_name = msg.category_name;
  result.score = msg.score;
  return result;
}

DetectronLabels detectronLabelsFromMsg(
    const panoptic_mapping_msgs::msg::DetectronLabels& msg) {
  DetectronLabels result;
  for (const panoptic_mapping_msgs::msg::DetectronLabel& label : msg.labels) {
    result[label.id] = detectronLabelFromMsg(label);
  }
  return result;
}

void convertToPointCloud2(
    const Pointcloud& points,
    const voxblox::AlignedVector<Eigen::Matrix<uint8_t, 3, 1>>& colors,
    sensor_msgs::msg::PointCloud2& cloud_msg) {
  if (points.empty() || colors.empty()) {
    return;
  }

  // 检查输入是否一致
  if (points.size() != colors.size()) {
    return;
  }

  // 初始化 PointCloud2 消息
  cloud_msg.height = 1;
  cloud_msg.width = points.size();
  cloud_msg.is_dense = true;
  cloud_msg.fields.resize(4);

  // 设置 x, y, z 字段
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[0].offset = 0;
  cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[0].count = 1;

  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[1].offset = 4;
  cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[1].count = 1;

  cloud_msg.fields[2].name = "z";
  cloud_msg.fields[2].offset = 8;
  cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[2].count = 1;

  // 设置 rgb 字段
  cloud_msg.fields[3].name = "rgb";
  cloud_msg.fields[3].offset = 16;
  cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
  cloud_msg.fields[3].count = 1;

  cloud_msg.is_bigendian = false;
  cloud_msg.point_step = 32;  // 每个点占 32 字节（x,y,z,rgb 各 4 字节）
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step);
  cloud_msg.is_dense = true;

  // 构造数据指针
  uint8_t* data_ptr = cloud_msg.data.data();

  for (size_t i = 0; i < points.size(); ++i) {
    float x = points[i].x();
    float y = points[i].y();
    float z = points[i].z();

    // 写入 xyz 数据（每个 float 占 4 字节）
    memcpy(data_ptr + 0, &x, sizeof(float));
    memcpy(data_ptr + 4, &y, sizeof(float));
    memcpy(data_ptr + 8, &z, sizeof(float));

    // 写入 RGB 颜色（打包成 32-bit 整数）
    uint32_t rgb = 0;
    if (!colors.empty()) {
      uint8_t r = colors[i][0];
      uint8_t g = colors[i][1];
      uint8_t b = colors[i][2];
      rgb = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) |
            static_cast<uint32_t>(b);
    } else {
      rgb = 0x00FFFFFF;  // 默认白色
    }
    memcpy(data_ptr + 16, &rgb, sizeof(uint32_t));

    data_ptr += cloud_msg.point_step;
  }
}

}  // namespace panoptic_mapping
