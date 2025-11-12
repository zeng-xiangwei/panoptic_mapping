#include "panoptic_mapping/common/input_data.h"

#include <opencv2/core/mat.hpp>

namespace panoptic_mapping {

// 拷贝构造函数 - 实现深拷贝
InputData::InputData(const InputData& other) 
    : T_M_C_(other.T_M_C_),
      sensor_frame_name_(other.sensor_frame_name_),
      timestamp_(other.timestamp_),
      detectron_labels_(other.detectron_labels_),
      contained_inputs_(other.contained_inputs_) {
  // 对OpenCV的Mat对象进行深拷贝
  if (!other.depth_image_.empty()) {
    depth_image_ = other.depth_image_.clone();
  }
  
  if (!other.color_image_.empty()) {
    color_image_ = other.color_image_.clone();
  }
  
  if (!other.id_image_.empty()) {
    id_image_ = other.id_image_.clone();
  }
  
  if (!other.uncertainty_image_.empty()) {
    uncertainty_image_ = other.uncertainty_image_.clone();
  }
  
  if (!other.id_image_copy_.empty()) {
    id_image_copy_ = other.id_image_copy_.clone();
  }
  
  if (!other.vertex_map_.empty()) {
    vertex_map_ = other.vertex_map_.clone();
  }
  
  if (!other.validity_image_.empty()) {
    validity_image_ = other.validity_image_.clone();
  }
}

// 拷贝赋值运算符 - 实现深拷贝
InputData& InputData::operator=(const InputData& other) {
  if (this != &other) {
    // 复制基本数据类型
    T_M_C_ = other.T_M_C_;
    sensor_frame_name_ = other.sensor_frame_name_;
    timestamp_ = other.timestamp_;
    detectron_labels_ = other.detectron_labels_;
    contained_inputs_ = other.contained_inputs_;
    
    // 对OpenCV的Mat对象进行深拷贝
    if (!other.depth_image_.empty()) {
      depth_image_ = other.depth_image_.clone();
    } else {
      depth_image_ = cv::Mat();
    }
    
    if (!other.color_image_.empty()) {
      color_image_ = other.color_image_.clone();
    } else {
      color_image_ = cv::Mat();
    }
    
    if (!other.id_image_.empty()) {
      id_image_ = other.id_image_.clone();
    } else {
      id_image_ = cv::Mat();
    }
    
    if (!other.uncertainty_image_.empty()) {
      uncertainty_image_ = other.uncertainty_image_.clone();
    } else {
      uncertainty_image_ = cv::Mat();
    }
    
    if (!other.id_image_copy_.empty()) {
      id_image_copy_ = other.id_image_copy_.clone();
    } else {
      id_image_copy_ = cv::Mat();
    }
    
    if (!other.vertex_map_.empty()) {
      vertex_map_ = other.vertex_map_.clone();
    } else {
      vertex_map_ = cv::Mat();
    }
    
    if (!other.validity_image_.empty()) {
      validity_image_ = other.validity_image_.clone();
    } else {
      validity_image_ = cv::Mat();
    }
  }
  
  return *this;
}

}  // namespace panoptic_mapping