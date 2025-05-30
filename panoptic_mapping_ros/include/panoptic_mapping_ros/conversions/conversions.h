#ifndef PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_
#define PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_

#include <panoptic_mapping/common/input_data.h>
#include <panoptic_mapping_msgs/DetectronLabel.h>
#include <panoptic_mapping_msgs/DetectronLabels.h>
#include <sensor_msgs/PointCloud2.h>

namespace panoptic_mapping {

DetectronLabel detectronLabelFromMsg(
    const panoptic_mapping_msgs::DetectronLabel& msg);

DetectronLabels detectronLabelsFromMsg(
    const panoptic_mapping_msgs::DetectronLabels& msg);

void convertToPointCloud2(
    const Pointcloud& points,
    const voxblox::AlignedVector<Eigen::Matrix<uint8_t, 3, 1>>& colors,
    sensor_msgs::PointCloud2& cloud_msg);

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_ROS_CONVERSIONS_CONVERSIONS_H_
