#include "panoptic_mapping/tools/image_data_manager.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "panoptic_mapping/tools/coloring.h"

namespace panoptic_mapping {

// ImageData拷贝构造函数
ImageData::ImageData(const ImageData& other)
    : image_id(other.image_id),
      timestamp(other.timestamp),
      rbg_image_file_name(other.rbg_image_file_name),
      id_image_file_name(other.id_image_file_name),
      associated_submaps(other.associated_submaps),
      is_processed(other.is_processed) {
  // 深拷贝图像数据
  if (!other.rgb_data.empty()) {
    rgb_data = other.rgb_data.clone();
  }
  if (!other.id_image_data.empty()) {
    id_image_data = other.id_image_data.clone();
  }
}

// ImageData拷贝赋值运算符
ImageData& ImageData::operator=(const ImageData& other) {
  if (this != &other) {
    image_id = other.image_id;
    timestamp = other.timestamp;
    rbg_image_file_name = other.rbg_image_file_name;
    id_image_file_name = other.id_image_file_name;
    associated_submaps = other.associated_submaps;
    is_processed = other.is_processed;

    // 深拷贝图像数据
    if (!other.rgb_data.empty()) {
      rgb_data = other.rgb_data.clone();
    } else {
      rgb_data.release();
    }

    if (!other.id_image_data.empty()) {
      id_image_data = other.id_image_data.clone();
    } else {
      id_image_data.release();
    }
  }
  return *this;
}

void ImageDataManager::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("image_save_directory", &image_save_directory);
  setupParam("max_images_in_memory", &max_images_in_memory);
  setupParam("enable_image_management", &enable_image_management);
  setupParam("load_image_data_info_on_startup",
             &load_image_data_info_on_startup);
  setupParam("meta_infos_file_name", &meta_infos_file_name);
  setupParam("min_IoU_for_box_matching", &min_IoU_for_box_matching);
  setupParam("vllm_middle_result_dir_name", &vllm_middle_result_dir_name);
}

void ImageDataManager::Config::checkParams() const {
  checkParamCond(!image_save_directory.empty(),
                 "'image_save_directory' may not be empty.");
}

ImageDataManager::ImageDataManager(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // 创建图像保存目录
  if (config_.enable_image_management) {
    std::filesystem::create_directories(config_.image_save_directory);
    std::filesystem::create_directories(getVllmMiddleResultsDir());
  }
}

void ImageDataManager::loadMap() {
  std::string meta_file_path = getImageDataInfoPath();
  LOG(INFO) << "Loading image data info from " << meta_file_path;
  loadMappingsFromFile(meta_file_path);
  initializeImageIdCounter();
}

void ImageDataManager::initializeImageIdCounter() {
  // 扫描保存目录中的文件，找出最大的ID
  int max_id = 0;

  for (auto& img_data : image_data_) {
    max_id = std::max(img_data.second->image_id, max_id);
  }

  current_image_id_ = max_id;
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Initialized image ID counter to " << current_image_id_;
}

int ImageDataManager::extractIdFromFilename(const std::string& filename) const {
  // 匹配模式: rbg_image_数字.png 或 id_image_数字.png 或 id_image_数字.bin
  std::regex pattern(R"(^(rbg_image_|id_image_)(\d+)(\.png|\.bin)$)");
  std::smatch matches;

  if (std::regex_match(filename, matches, pattern) && matches.size() > 2) {
    try {
      return std::stoi(matches[2].str());
    } catch (const std::exception& e) {
      // 转换失败，返回0
    }
  }

  return 0;
}

int ImageDataManager::addImageData(const cv::Mat& image,
                                   const cv::Mat& id_image,
                                   const DetectronLabels& detectron_labels,
                                   double timestamp,
                                   const SubmapCollection& submaps) {
  if (!config_.enable_image_management) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // 检查是否有submap变化，决定是否添加图像
  if (!needToRetainImageData(submaps)) {
    LOG_IF(INFO, config_.verbosity >= 3)
        << "No submap changes detected, skipping image data addition.";
    return -1;  // 表示未添加图像
  }

  // 创建新的图像数据（仅元数据）
  auto image_data = std::make_shared<ImageData>();
  image_data->image_id = ++current_image_id_;
  image_data->timestamp = timestamp;
  image_data->rbg_image_file_name =
      "rbg_image_" + std::to_string(image_data->image_id) + ".png";
  image_data->id_image_file_name =
      "id_image_" + std::to_string(image_data->image_id) + ".bin";

  // 直接保存图像到磁盘
  if (!saveImageToDisk(image, id_image, image_data->rbg_image_file_name,
                       image_data->id_image_file_name)) {
    LOG_IF(ERROR, config_.verbosity >= 1)
        << "Failed to save image data with ID " << image_data->image_id
        << " to disk.";
    return -1;
  }

  // 保存到内存（仅元数据）
  image_data_[image_data->image_id] = image_data;
  // 自动关联图像中的submap
  autoAssociateSubmaps(image_data->image_id, id_image, submaps);
  unprocessed_images_.push(image_data->image_id);

  // 创建包含实际图像数据的缓存副本
  auto cache_data = std::make_shared<ImageData>(*image_data);
  cache_data->rgb_data = image.clone();
  cache_data->id_image_data = id_image.clone();
  addToCache(cache_data);

  LOG_IF(INFO, config_.verbosity >= 2)
      << "Added image data with ID " << image_data->image_id
      << " and associated with " << image_data->associated_submaps.size()
      << " submaps.";

  // 处理已经删除的 submap
  getAndRemoveSubmap(submaps);

  return image_data->image_id;
}

void ImageDataManager::getAndRemoveSubmap(const SubmapCollection& submaps) {
  // 处理已经删除的 submap
  std::vector<int> deleted_submap_ids;
  getDeletedSubmaps(submaps, deleted_submap_ids);
  for (int submap_id : deleted_submap_ids) {
    handleSubmapRemoval(submap_id);
    if (last_added_active_submaps_.count(submap_id) > 0) {
      last_added_active_submaps_.erase(submap_id);
    }
  }
}

void ImageDataManager::getAndRemoveSubmapUnderLock(
    const SubmapCollection& submaps) {
  if (!config_.enable_image_management) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  getAndRemoveSubmap(submaps);
}

bool ImageDataManager::needToRetainImageData(const SubmapCollection& submaps) {
  std::unordered_set<int> activate_submap_ids;
  for (const Submap& submap : submaps) {
    if (submap.isActive() && submap.matchRedetection() &&
        submap.getLabel() == PanopticLabel::kInstance) {
      // 满足重复观测条件的活跃 submap
      activate_submap_ids.insert(submap.getID());
    }
  }

  bool need_to_retain = false;
  for (auto k : activate_submap_ids) {
    if (last_added_active_submaps_.count(k) == 0) {
      LOG(INFO) << "Submap " << k
                << " is new valid object, will add this image";
      need_to_retain = true;
      break;
    }
  }

  for (auto k : activate_submap_ids) {
    last_added_active_submaps_.insert(k);
  }

  return need_to_retain;
}

void ImageDataManager::getDeletedSubmaps(const SubmapCollection& submaps,
                                         std::vector<int>& deleted_submap_ids) {
  std::vector<int> ids(last_submap_ids_.begin(), last_submap_ids_.end());
  std::vector<int> new_ids;
  submaps.updateIDList(ids, &new_ids, &deleted_submap_ids);
  for (int deleted_id : deleted_submap_ids) {
    last_submap_ids_.erase(deleted_id);
  }
  for (int new_id : new_ids) {
    last_submap_ids_.insert(new_id);
  }
}

std::shared_ptr<ImageData> ImageDataManager::getImageData(int image_id) {
  if (!config_.enable_image_management) {
    return nullptr;
  }

  // 首先检查缓存中是否有数据
  auto cache_it = image_cache_.find(image_id);
  if (cache_it != image_cache_.end()) {
    // 创建深拷贝副本
    auto copy_data = std::make_shared<ImageData>(*cache_it->second);

    // 更新缓存访问历史
    updateCacheAccess(image_id);

    LOG_IF(INFO, config_.verbosity >= 3)
        << "Retrieved image data with ID " << image_id << " from cache.";
    return copy_data;
  }

  // 缓存中没有，检查元数据是否存在
  auto it = image_data_.find(image_id);
  if (it == image_data_.end()) {
    LOG_IF(WARNING, config_.verbosity >= 1)
        << "Image data with ID " << image_id << " not found.";
    return nullptr;
  }

  // 创建新的缓存数据
  auto cache_data = std::make_shared<ImageData>(*it->second);

  // 加载图像数据
  if (loadImageFromDisk(cache_data)) {
    // 创建深拷贝副本
    auto copy_data = std::make_shared<ImageData>(*cache_data);

    // 添加到缓存
    addToCache(cache_data);

    LOG_IF(INFO, config_.verbosity >= 3)
        << "Loaded and retrieved image data with ID " << image_id
        << " from disk.";
    return copy_data;
  }

  LOG_IF(WARNING, config_.verbosity >= 1)
      << "Failed to load image data with ID " << image_id << " from disk.";
  return nullptr;
}

std::shared_ptr<ImageData>
ImageDataManager::getFirstNotProcessedImageDataForVLLM() {
  std::lock_guard<std::mutex> lock(mutex_);

  // 检查是否有未处理的图像
  if (unprocessed_images_.empty()) {
    return nullptr;
  }

  // 获取第一个未处理的图像ID
  int image_id = unprocessed_images_.front();

  // 获取图像数据
  auto image_data = getImageData(image_id);
  if (!image_data) {
    // 如果获取失败，从未处理集合中移除
    unprocessed_images_.pop();
    return nullptr;
  }

  unprocessed_images_.pop();

  return image_data;
}

std::unordered_map<int, std::unordered_map<int, float>>
ImageDataManager::associateSubmapAndVLLMBBox(
    const VLLMOutputData& vllm_output, std::shared_ptr<ImageData> image_data) {
  std::unordered_map<int, std::unordered_map<int, float>>
      box_submap_associations;

  bool use_box_id_as_submap_id = false;
  for (const auto& bbox_info : vllm_output.bounding_boxes_info) {
    if (bbox_info.box_id_is_submap_id) {
      use_box_id_as_submap_id = true;
      break;
    }
  }

  // 直接使用box ID作为submap ID进行关联（如果启用该选项）
  if (use_box_id_as_submap_id) {
    LOG(INFO) << "Using box ID as submap ID for association.";
    for (const auto& bbox_info : vllm_output.bounding_boxes_info) {
      if (bbox_info.box_id_is_submap_id) {
        int submap_id = bbox_info.id;
        box_submap_associations[bbox_info.id][submap_id] =
            1.0f;  // 假设完全匹配
      }
    }
    return box_submap_associations;
  }

  // 统计 box 与 submap 的匹配关系，用来构造 box 和 submap 的一对一关系 <box_id,
  // <submap_id, IoU>>
  const auto& submaps_in_image = image_data->associated_submaps;
  for (const auto& bbox_info : vllm_output.bounding_boxes_info) {
    const cv::Rect& vllm_bbox = bbox_info.bounding_box;
    std::string bbox_class_name = bbox_info.description.class_name;

    // 计算每个关联submap与边界框的IoU
    std::unordered_map<int, float> submap_ious;
    for (const auto& [submap_id, submap_data] : submaps_in_image) {
      std::string submap_class_name = submap_data.class_name;
      if (submap_class_name != bbox_class_name) {
        continue;
      }

      cv::Rect submap_rect = submap_data.bounding_box;

      // 计算两个边界框的交集区域
      cv::Rect intersection = submap_rect & vllm_bbox;

      // 计算各个区域面积
      float submap_area = submap_rect.area();
      float detected_area = vllm_bbox.area();
      float intersection_area = intersection.area();

      // 计算IoU
      if (intersection_area > 0) {
        float union_area = submap_area + detected_area - intersection_area;
        float iou = intersection_area / union_area;
        submap_ious[submap_id] = iou;

        if (config_.verbosity >= 3) {
          LOG(INFO) << "Submap " << submap_id << " vs detection box "
                    << bbox_info.id << " IoU: " << iou
                    << " (Intersection: " << intersection_area
                    << ", Union: " << union_area << ")";
        }
      }
    }

    if (submap_ious.empty()) {
      continue;
    }

    // 找到具有最高IoU的submap
    int best_submap_id = -1;
    float max_iou = -1.0f;

    for (const auto& pair : submap_ious) {
      if (pair.second > max_iou) {
        max_iou = pair.second;
        best_submap_id = pair.first;
      }
    }

    // 如果找到了匹配的submap且IoU足够高，则更新submap信息
    if (best_submap_id != -1 && max_iou > config_.min_IoU_for_box_matching) {
      LOG(INFO) << "Associating bounding box " << bbox_info.id
                << " with submap " << best_submap_id << " (IoU: " << max_iou
                << ")";

      box_submap_associations[bbox_info.id][best_submap_id] = max_iou;
    }
  }

  return box_submap_associations;
}

void ImageDataManager::processVLLMOutput(const VLLMOutputData& vllm_output,
                                         SubmapCollection& submaps) {
  std::lock_guard<std::mutex> lock(mutex_);

  // 获取对应的图像数据
  auto image_data = getImageData(vllm_output.image_id);
  if (!image_data) {
    LOG(WARNING) << "VLLM output references non-existent image ID: "
                 << vllm_output.image_id;
    return;
  }

  // 标记图像为已处理
  markImageAsProcessed(vllm_output.image_id);

  // 统计 box 与 submap 的匹配关系，用来构造 box 和 submap 的一对一关系 <box_id,
  // <submap_id, IoU>>
  std::unordered_map<int, std::unordered_map<int, float>>
      box_submap_associations =
          associateSubmapAndVLLMBBox(vllm_output, image_data);

  // 取 box 和 submap 一一对应的结果，构造 submap 之间的关系
  // TODO: 可能出现 1 个 submap 对应多个 box
  // 的情况，暂时不做处理，对于物体的描述，直接做替换更新；对于物体与物体的关系，每个box都贡献一个关系
  std::unordered_map<int, int> box_submap_pair;
  for (const auto& [box_id, submap_id_and_iou] : box_submap_associations) {
    float max_iou = -1.0f;
    int best_submap_id = -1;
    for (const auto& [submap_id, iou] : submap_id_and_iou) {
      if (iou > max_iou) {
        max_iou = iou;
        best_submap_id = submap_id;
      }
    }
    if (best_submap_id != -1) {
      box_submap_pair[box_id] = best_submap_id;
    }
  }

  updateSubmap(vllm_output, box_submap_pair, submaps);

  visualVllmOutput(vllm_output, image_data, box_submap_pair);

  LOG(INFO) << "Processed VLLM output for image ID: " << vllm_output.image_id
            << " with " << vllm_output.bounding_boxes_info.size()
            << " bounding boxes";
}

void ImageDataManager::updateSubmap(
    const VLLMOutputData& vllm_output,
    std::unordered_map<int, int> box_submap_pair, SubmapCollection& submaps) {
  for (const auto& box_info : vllm_output.bounding_boxes_info) {
    if (box_submap_pair.count(box_info.id) == 0) {
      continue;
    }
    int box_id = box_info.id;
    int submap_id = box_submap_pair[box_id];
    if (!submaps.submapIdExists(submap_id)) {
      continue;
    }
    Submap* submap = submaps.getSubmapPtr(submap_id);
    submap->setDescriptsByVllm(box_info.description);
    submap->setHasNewVllmDescripts(true);
    generated_vllm_desc_submap_ids_.insert(submap_id);
    LOG(INFO) << "Updating submap " << submap->getID()
              << " with bounding box (id:" << box_info.id
              << ") (Rect: " << box_info.bounding_box << ")"
              << ", and description: ["
              << submap->getDescriptsByVllm().toString() << "]";
  }

  for (const auto& box_relationship :
       vllm_output.bounding_boxes_relationships) {
    if (box_submap_pair.count(box_relationship.from_id) == 0 ||
        box_submap_pair.count(box_relationship.to_id) == 0) {
      continue;
    }

    int box_from_id = box_relationship.from_id;
    int box_to_id = box_relationship.to_id;
    int submap_from_id = box_submap_pair[box_from_id];
    int submap_to_id = box_submap_pair[box_to_id];
    RelationshipType relationship_type = box_relationship.relationship;

    if (!submaps.submapIdExists(submap_from_id)) {
      LOG(WARNING) << "Submap " << submap_from_id
                   << " does not exist in submap collection when processing "
                      "relationship from VLLM output for image ID: "
                   << vllm_output.image_id;
      continue;
    }
    if (!submaps.submapIdExists(submap_to_id)) {
      LOG(WARNING) << "Submap " << submap_to_id
                   << " does not exist in submap collection when processing "
                      "relationship from VLLM output for image ID: "
                   << vllm_output.image_id;
      continue;
    }

    Submap* submap_from = submaps.getSubmapPtr(submap_from_id);
    Submap* submap_to = submaps.getSubmapPtr(submap_to_id);

    VllmRelationship from_relationship;
    from_relationship.from_id = submap_from_id;
    from_relationship.to_id = submap_to_id;
    from_relationship.relationship = relationship_type;
    submap_from->getVllmRelationshipsPtr()->push_back(from_relationship);
    submap_from->setHasNewVllmDescripts(true);

    VllmRelationship to_relationship;
    to_relationship.from_id = submap_to_id;
    to_relationship.to_id = submap_from_id;
    to_relationship.relationship = inverseRelationshipType(relationship_type);
    submap_from->getVllmRelationshipsPtr()->push_back(to_relationship);
    submap_from->setHasNewVllmDescripts(true);
  }
}

void ImageDataManager::markImageAsProcessed(int image_id) {
  if (!config_.enable_image_management) {
    return;
  }

  auto it = image_data_.find(image_id);
  if (it != image_data_.end()) {
    it->second->is_processed = true;
    LOG_IF(INFO, config_.verbosity >= 2)
        << "Marked image " << image_id << " as processed.";
  }
}

void ImageDataManager::associateSubmapWithImage(
    int submap_id, int image_id, const Submap& submap,
    const Eigen::Vector4i& bounding_box) {
  if (!config_.enable_image_management) {
    return;
  }

  // 建立双向关联
  submap_to_images_[submap_id].insert(image_id);
  image_to_submaps_[image_id].insert(submap_id);

  // 在图像数据中也记录关联
  auto it = image_data_.find(image_id);
  if (it != image_data_.end()) {
    SubmapData submap_data;
    submap_data.submap_id = submap_id;
    submap_data.bounding_box = cv::Rect(bounding_box[0], bounding_box[1],
                                        bounding_box[2] - bounding_box[0] + 1,
                                        bounding_box[3] - bounding_box[1] + 1);
    submap_data.class_name = submap.getClassName();
    it->second->associated_submaps[submap_id] = submap_data;
    LOG_IF(INFO, config_.verbosity >= 3)
        << "Associated submap " << submap_id << " with image " << image_id;
  }
}

void ImageDataManager::dissociateSubmapFromImage(int submap_id, int image_id) {
  if (!config_.enable_image_management) {
    return;
  }

  // 解除双向关联
  auto submap_it = submap_to_images_.find(submap_id);
  if (submap_it != submap_to_images_.end()) {
    submap_it->second.erase(image_id);
    if (submap_it->second.empty()) {
      submap_to_images_.erase(submap_it);
    }
  }

  auto image_it = image_to_submaps_.find(image_id);
  if (image_it != image_to_submaps_.end()) {
    image_it->second.erase(submap_id);
    if (image_it->second.empty()) {
      image_to_submaps_.erase(image_it);
    }
  }

  // 在图像数据中也移除关联
  auto data_it = image_data_.find(image_id);
  if (data_it != image_data_.end()) {
    data_it->second->associated_submaps.erase(submap_id);
  }

  LOG_IF(INFO, config_.verbosity >= 3)
      << "Dissociated submap " << submap_id << " from image " << image_id;
}

std::shared_ptr<ImageData> ImageDataManager::getImageForSubmap(int submap_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::shared_ptr<ImageData> result = nullptr;
  auto it = submap_to_images_.find(submap_id);
  if (it != submap_to_images_.end()) {
    // 获取该 submaps 最新的 image
    int image_id = *it->second.rbegin();
    // 返回的是ImageData带有图像数据的深拷贝结果，因此直接返回即可
    result = getImageData(image_id);
  }

  return result;
}

void ImageDataManager::handleSubmapRemoval(int submap_id) {
  if (!config_.enable_image_management) {
    return;
  }

  // 获取与该submap关联的所有图像
  auto it = submap_to_images_.find(submap_id);
  if (it == submap_to_images_.end()) {
    return;
  }

  // 解除所有关联
  std::vector<int> associated_images(it->second.begin(), it->second.end());
  for (int image_id : associated_images) {
    dissociateSubmapFromImage(submap_id, image_id);
  }

  // 清理不关联任何submap的图像数据
  cleanupUnassociatedImages();

  LOG_IF(INFO, config_.verbosity >= 2)
      << "Handled removal of submap " << submap_id << ", dissociated from "
      << associated_images.size() << " images.";
}

void ImageDataManager::cleanupUnassociatedImages() {
  if (!config_.enable_image_management) {
    return;
  }

  std::vector<int> images_to_remove;
  for (const auto& pair : image_data_) {
    // 如果图像不关联任何submap，则可以删除
    if (pair.second->associated_submaps.empty()) {
      images_to_remove.push_back(pair.first);
    }
  }

  int removed_count = 0;
  for (int image_id : images_to_remove) {
    // 从缓存中移除
    removeFromCache(image_id);

    // 删除磁盘上的文件
    auto it = image_data_.find(image_id);
    if (it != image_data_.end()) {
      deleteImageFromDisk(it->second);

      // 从内存中移除
      image_data_.erase(it);
      removed_count++;
    }
  }

  LOG_IF(INFO, config_.verbosity >= 2)
      << "Cleaned up " << removed_count << " unassociated images.";
}

bool ImageDataManager::saveImageToDisk(const cv::Mat& image,
                                       const cv::Mat& id_image,
                                       const std::string& rgb_image_name,
                                       const std::string& id_image_name) {
  // 创建文件路径
  std::string image_file = getImagePath(rgb_image_name);
  std::string id_image_file = getImagePath(id_image_name);

  try {
    // 保存RGB图像为PNG
    bool image_saved = cv::imwrite(image_file, image);

    // 保存ID图像为二进制文件
    bool id_image_saved = false;
    std::ofstream id_file(id_image_file, std::ios::binary);
    if (id_file.is_open()) {
      // 写入图像尺寸信息
      int rows = id_image.rows;
      int cols = id_image.cols;
      id_file.write(reinterpret_cast<const char*>(&rows), sizeof(int32_t));
      id_file.write(reinterpret_cast<const char*>(&cols), sizeof(int32_t));

      // 写入图像数据
      id_file.write(reinterpret_cast<const char*>(id_image.data),
                    rows * cols * sizeof(int32_t));
      id_file.close();
      id_image_saved = true;
    }

    return image_saved && id_image_saved;
  } catch (const std::exception& e) {
    LOG_IF(ERROR, config_.verbosity >= 1)
        << "Failed to save image (" << image_file << ", " << id_image_file
        << ") to disk: " << e.what();
  }

  return false;
}

bool ImageDataManager::loadImageFromDisk(
    const std::shared_ptr<ImageData>& image_data) {
  if (image_data->rbg_image_file_name.empty() ||
      image_data->id_image_file_name.empty()) {
    return false;
  }

  try {
    // 从磁盘加载RGB图像
    image_data->rgb_data =
        cv::imread(getImagePath(image_data->rbg_image_file_name));

    // 从磁盘加载ID图像（二进制文件）
    std::ifstream id_file(getImagePath(image_data->id_image_file_name),
                          std::ios::binary);
    if (id_file.is_open()) {
      // 读取图像尺寸信息
      int rows, cols;
      id_file.read(reinterpret_cast<char*>(&rows), sizeof(int32_t));
      id_file.read(reinterpret_cast<char*>(&cols), sizeof(int32_t));

      // 创建图像矩阵
      image_data->id_image_data = cv::Mat(rows, cols, CV_32SC1);

      // 读取图像数据
      id_file.read(reinterpret_cast<char*>(image_data->id_image_data.data),
                   rows * cols * sizeof(int32_t));
      id_file.close();
    }

    if (!image_data->rgb_data.empty() && !image_data->id_image_data.empty()) {
      LOG_IF(INFO, config_.verbosity >= 3)
          << "Loaded image " << image_data->image_id << " from disk.";
      return true;
    }
  } catch (const std::exception& e) {
    LOG_IF(ERROR, config_.verbosity >= 1)
        << "Failed to load image " << image_data->image_id
        << " from disk: " << e.what();
  }

  return false;
}

bool ImageDataManager::deleteImageFromDisk(
    const std::shared_ptr<ImageData>& image_data) {
  try {
    bool image_deleted = true;
    bool id_image_deleted = true;

    // 删除RGB图像文件
    if (!image_data->rbg_image_file_name.empty()) {
      image_deleted = std::filesystem::remove(
          getImagePath(image_data->rbg_image_file_name));
    }

    // 删除ID图像文件
    if (!image_data->id_image_file_name.empty()) {
      id_image_deleted =
          std::filesystem::remove(getImagePath(image_data->id_image_file_name));
    }

    LOG_IF(INFO, config_.verbosity >= 3)
        << "Deleted image " << image_data->image_id << " from disk.";
    return image_deleted && id_image_deleted;
  } catch (const std::exception& e) {
    LOG_IF(ERROR, config_.verbosity >= 1)
        << "Failed to delete image " << image_data->image_id
        << " from disk: " << e.what();
  }

  return false;
}

void ImageDataManager::autoAssociateSubmaps(int image_id,
                                            const cv::Mat& id_image,
                                            const SubmapCollection& submaps) {
  // 从id_image中提取所有唯一的submap ID
  std::set<int> unique_ids;
  for (int y = 0; y < id_image.rows; ++y) {
    for (int x = 0; x < id_image.cols; ++x) {
      int id = id_image.at<int>(y, x);
      if (id >= 0) {  // 有效的submap ID是非负数
        unique_ids.insert(id);
      }
    }
  }

  // 根据 mask 图像，计算每个 submap 的 box，vector 中存储的是 xmin ymin xmax
  // ymax
  std::unordered_map<int, Eigen::Vector4i> submap_with_boxes;
  for (int x = 0; x < id_image.cols; ++x) {
    for (int y = 0; y < id_image.rows; ++y) {
      int submap_id = id_image.at<int32_t>(y, x);
      if (submap_id < 0) {
        continue;
      }

      if (submap_with_boxes.count(submap_id) == 0) {
        int int_max = std::numeric_limits<int>::max();
        submap_with_boxes[submap_id] = Eigen::Vector4i(int_max, int_max, 0, 0);
      }
      Eigen::Vector4i& bbox = submap_with_boxes[submap_id];
      bbox[0] = std::min(bbox[0], x);
      bbox[1] = std::min(bbox[1], y);
      bbox[2] = std::max(bbox[2], x);
      bbox[3] = std::max(bbox[3], y);
    }
  }

  // 关联所有在当前submap集合中存在的ID
  for (int submap_id : unique_ids) {
    if (submaps.submapIdExists(submap_id)) {
      const Submap& submap = submaps.getSubmap(submap_id);
      if (submap.getLabel() != PanopticLabel::kInstance) {
        continue;
      }
      if (submap_with_boxes.count(submap_id) == 0) {
        continue;
      }
      associateSubmapWithImage(submap_id, image_id, submap,
                               submap_with_boxes[submap_id]);
    }
  }
}

void ImageDataManager::addToCache(
    const std::shared_ptr<ImageData>& image_data) {
  // 检查该图像是否在主数据存储中存在
  if (image_data_.find(image_data->image_id) == image_data_.end()) {
    LOG_IF(WARNING, config_.verbosity >= 1)
        << "Trying to add image " << image_data->image_id
        << " to cache, but it doesn't exist in main storage.";
    return;
  }

  // 检查缓存是否已满，如果满则驱逐
  evictCacheIfNeeded();

  // 添加到缓存
  image_cache_[image_data->image_id] = image_data;
  image_access_history_.push_front(image_data->image_id);

  LOG_IF(INFO, config_.verbosity >= 3)
      << "Added image " << image_data->image_id << " to cache.";
}

void ImageDataManager::removeFromCache(int image_id) {
  auto it = image_cache_.find(image_id);
  if (it != image_cache_.end()) {
    image_cache_.erase(it);

    // 从访问历史中移除
    image_access_history_.remove(image_id);

    LOG_IF(INFO, config_.verbosity >= 3)
        << "Removed image " << image_id << " from cache.";
  }
}

void ImageDataManager::updateCacheAccess(int image_id) {
  // 将该图像ID移到访问历史的前面
  // TODO:
  // 缓存考虑访问次数+时间，次数越低优先级越低，时间越久优先级越低；
  // 还需要考虑区分第一个输入的图像与查询时获取的图像的优先级是不同的
  image_access_history_.remove(image_id);
  image_access_history_.push_front(image_id);
}

void ImageDataManager::evictCacheIfNeeded() {
  if (image_cache_.size() >= config_.max_images_in_memory) {
    // 驱逐最少使用的图像（LRU策略）
    while (image_cache_.size() >= config_.max_images_in_memory &&
           !image_access_history_.empty()) {
      int lru_image_id = image_access_history_.back();
      image_access_history_.pop_back();

      auto it = image_cache_.find(lru_image_id);
      if (it != image_cache_.end()) {
        image_cache_.erase(it);

        LOG_IF(INFO, config_.verbosity >= 3)
            << "Evicted image " << lru_image_id << " from cache.";
      }
    }
  }
}

std::shared_ptr<ImageData> ImageDataManager::createMetadataCopy(
    const std::shared_ptr<ImageData>& source) {
  auto copy = std::make_shared<ImageData>();
  copy->image_id = source->image_id;
  copy->timestamp = source->timestamp;
  copy->rbg_image_file_name = source->rbg_image_file_name;
  copy->id_image_file_name = source->id_image_file_name;
  copy->associated_submaps = source->associated_submaps;
  copy->is_processed = source->is_processed;
  return copy;
}

void ImageDataManager::saveMappingsToFile(const std::string& filepath) const {
  std::ofstream file(filepath, std::ios::binary);
  if (!file.is_open()) {
    LOG_IF(ERROR, config_.verbosity >= 1)
        << "Failed to open file for writing mappings: " << filepath;
    return;
  }

  // 写入文件版本标识
  uint32_t version = 1;
  file.write(reinterpret_cast<const char*>(&version), sizeof(version));

  // 保存 image_data_ 映射
  uint32_t image_data_count = image_data_.size();
  file.write(reinterpret_cast<const char*>(&image_data_count),
             sizeof(image_data_count));

  for (const auto& pair : image_data_) {
    int image_id = pair.first;
    const std::shared_ptr<ImageData>& image_data = pair.second;

    file.write(reinterpret_cast<const char*>(&image_id), sizeof(image_id));
    file.write(reinterpret_cast<const char*>(&image_data->timestamp),
               sizeof(image_data->timestamp));

    // 写入字符串长度和字符串内容
    uint32_t image_file_path_length = image_data->rbg_image_file_name.length();
    file.write(reinterpret_cast<const char*>(&image_file_path_length),
               sizeof(image_file_path_length));
    file.write(image_data->rbg_image_file_name.c_str(), image_file_path_length);

    uint32_t id_image_file_path_length =
        image_data->id_image_file_name.length();
    file.write(reinterpret_cast<const char*>(&id_image_file_path_length),
               sizeof(id_image_file_path_length));
    file.write(image_data->id_image_file_name.c_str(),
               id_image_file_path_length);

    // 写入关联的submap数量
    uint32_t associated_submaps_count = image_data->associated_submaps.size();
    file.write(reinterpret_cast<const char*>(&associated_submaps_count),
               sizeof(associated_submaps_count));

    // 写入每个关联的submap ID及其SubmapData
    for (const auto& submap_pair : image_data->associated_submaps) {
      int submap_id = submap_pair.first;
      const SubmapData& submap_data = submap_pair.second;

      file.write(reinterpret_cast<const char*>(&submap_id), sizeof(submap_id));

      // 写入类名字符串长度和内容
      uint32_t class_name_length = submap_data.class_name.length();
      file.write(reinterpret_cast<const char*>(&class_name_length),
                 sizeof(class_name_length));
      file.write(submap_data.class_name.c_str(), class_name_length);

      // 写入bounding box信息 (x, y, width, height)
      file.write(reinterpret_cast<const char*>(&submap_data.bounding_box.x),
                 sizeof(submap_data.bounding_box.x));
      file.write(reinterpret_cast<const char*>(&submap_data.bounding_box.y),
                 sizeof(submap_data.bounding_box.y));
      file.write(reinterpret_cast<const char*>(&submap_data.bounding_box.width),
                 sizeof(submap_data.bounding_box.width));
      file.write(
          reinterpret_cast<const char*>(&submap_data.bounding_box.height),
          sizeof(submap_data.bounding_box.height));
    }

    // 写入处理状态
    file.write(reinterpret_cast<const char*>(&image_data->is_processed),
               sizeof(image_data->is_processed));
  }

  // 保存 submap_to_images_ 映射
  uint32_t submap_count = submap_to_images_.size();
  file.write(reinterpret_cast<const char*>(&submap_count),
             sizeof(submap_count));

  for (const auto& pair : submap_to_images_) {
    int submap_id = pair.first;
    const auto& image_ids = pair.second;

    file.write(reinterpret_cast<const char*>(&submap_id), sizeof(submap_id));

    uint32_t image_count = image_ids.size();
    file.write(reinterpret_cast<const char*>(&image_count),
               sizeof(image_count));

    for (int image_id : image_ids) {
      file.write(reinterpret_cast<const char*>(&image_id), sizeof(image_id));
    }
  }

  // 保存 image_to_submaps_ 映射
  uint32_t image_count = image_to_submaps_.size();
  file.write(reinterpret_cast<const char*>(&image_count), sizeof(image_count));

  for (const auto& pair : image_to_submaps_) {
    int image_id = pair.first;
    const std::unordered_set<int>& submap_ids = pair.second;

    file.write(reinterpret_cast<const char*>(&image_id), sizeof(image_id));

    uint32_t submap_count_inner = submap_ids.size();
    file.write(reinterpret_cast<const char*>(&submap_count_inner),
               sizeof(submap_count_inner));

    for (int submap_id : submap_ids) {
      file.write(reinterpret_cast<const char*>(&submap_id), sizeof(submap_id));
    }
  }

  file.close();

  LOG_IF(INFO, config_.verbosity >= 2)
      << "Saved mappings to " << filepath
      << " (image data: " << image_data_.size()
      << ", submap mappings: " << submap_to_images_.size()
      << ", image mappings: " << image_to_submaps_.size() << ")";
}

void ImageDataManager::loadMappingsFromFile(const std::string& filepath) {
  std::ifstream file(filepath, std::ios::binary);
  if (!file.is_open()) {
    LOG_IF(WARNING, config_.verbosity >= 2)
        << "Failed to open file for reading mappings: " << filepath
        << " (file may not exist yet)";
    return;
  }

  // 清空现有映射
  image_data_.clear();
  submap_to_images_.clear();
  image_to_submaps_.clear();

  // 读取文件版本标识
  uint32_t version;
  file.read(reinterpret_cast<char*>(&version), sizeof(version));

  if (version != 1) {
    LOG_IF(ERROR, config_.verbosity >= 1)
        << "Unsupported mappings file version: " << version;
    file.close();
    return;
  }

  // 读取 image_data_ 映射
  uint32_t image_data_count;
  file.read(reinterpret_cast<char*>(&image_data_count),
            sizeof(image_data_count));

  for (uint32_t i = 0; i < image_data_count; ++i) {
    auto image_data = std::make_shared<ImageData>();

    file.read(reinterpret_cast<char*>(&image_data->image_id),
              sizeof(image_data->image_id));
    file.read(reinterpret_cast<char*>(&image_data->timestamp),
              sizeof(image_data->timestamp));

    // 读取图像文件路径
    uint32_t image_file_path_length;
    file.read(reinterpret_cast<char*>(&image_file_path_length),
              sizeof(image_file_path_length));
    image_data->rbg_image_file_name.resize(image_file_path_length);
    file.read(&image_data->rbg_image_file_name[0], image_file_path_length);

    // 读取ID图像文件路径
    uint32_t id_image_file_path_length;
    file.read(reinterpret_cast<char*>(&id_image_file_path_length),
              sizeof(id_image_file_path_length));
    image_data->id_image_file_name.resize(id_image_file_path_length);
    file.read(&image_data->id_image_file_name[0], id_image_file_path_length);

    // 读取关联的submap ID及SubmapData
    uint32_t associated_submaps_count;
    file.read(reinterpret_cast<char*>(&associated_submaps_count),
              sizeof(associated_submaps_count));

    for (uint32_t j = 0; j < associated_submaps_count; ++j) {
      int submap_id;
      file.read(reinterpret_cast<char*>(&submap_id), sizeof(submap_id));

      // 读取类名
      uint32_t class_name_length;
      file.read(reinterpret_cast<char*>(&class_name_length),
                sizeof(class_name_length));
      std::string class_name(class_name_length, '\0');
      file.read(&class_name[0], class_name_length);

      // 读取bounding box信息
      cv::Rect bounding_box;
      file.read(reinterpret_cast<char*>(&bounding_box.x),
                sizeof(bounding_box.x));
      file.read(reinterpret_cast<char*>(&bounding_box.y),
                sizeof(bounding_box.y));
      file.read(reinterpret_cast<char*>(&bounding_box.width),
                sizeof(bounding_box.width));
      file.read(reinterpret_cast<char*>(&bounding_box.height),
                sizeof(bounding_box.height));

      // 构造SubmapData并插入到associated_submaps中
      SubmapData submap_data;
      submap_data.submap_id = submap_id;
      submap_data.class_name = class_name;
      submap_data.bounding_box = bounding_box;

      image_data->associated_submaps[submap_id] = submap_data;
    }

    // 读取处理状态
    file.read(reinterpret_cast<char*>(&image_data->is_processed),
              sizeof(image_data->is_processed));

    // 将数据添加到映射中
    image_data_[image_data->image_id] = image_data;
  }

  // 读取 submap_to_images_ 映射
  uint32_t submap_count;
  file.read(reinterpret_cast<char*>(&submap_count), sizeof(submap_count));

  for (uint32_t i = 0; i < submap_count; ++i) {
    int submap_id;
    file.read(reinterpret_cast<char*>(&submap_id), sizeof(submap_id));

    uint32_t image_count;
    file.read(reinterpret_cast<char*>(&image_count), sizeof(image_count));

    for (uint32_t j = 0; j < image_count; ++j) {
      int image_id;
      file.read(reinterpret_cast<char*>(&image_id), sizeof(image_id));
      submap_to_images_[submap_id].insert(image_id);
    }
  }

  // 读取 image_to_submaps_ 映射
  uint32_t image_count;
  file.read(reinterpret_cast<char*>(&image_count), sizeof(image_count));

  for (uint32_t i = 0; i < image_count; ++i) {
    int image_id;
    file.read(reinterpret_cast<char*>(&image_id), sizeof(image_id));

    uint32_t submap_count_inner;
    file.read(reinterpret_cast<char*>(&submap_count_inner),
              sizeof(submap_count_inner));

    for (uint32_t j = 0; j < submap_count_inner; ++j) {
      int submap_id;
      file.read(reinterpret_cast<char*>(&submap_id), sizeof(submap_id));
      image_to_submaps_[image_id].insert(submap_id);
    }
  }

  file.close();

  LOG_IF(INFO, config_.verbosity >= 2)
      << "Loaded mappings from " << filepath
      << " (image data: " << image_data_.size()
      << ", submap mappings: " << submap_to_images_.size()
      << ", image mappings: " << image_to_submaps_.size() << ")";
}

std::string ImageDataManager::getImagePath(const std::string& file_name) const {
  return config_.image_save_directory + "/" + file_name;
}

std::string ImageDataManager::getImageDataInfoPath() const {
  return config_.image_save_directory + "/" + config_.meta_infos_file_name;
}

std::string ImageDataManager::getVllmMiddleResultsDir() const {
  return config_.image_save_directory + "/" +
         config_.vllm_middle_result_dir_name;
}

void ImageDataManager::visualVllmOutput(
    const VLLMOutputData& vllm_output, std::shared_ptr<ImageData> image_data,
    std::unordered_map<int, int> box_submap_pair) {
  // 创建RGB图像的副本用于绘制
  cv::Mat visualization_image;
  image_data->rgb_data.copyTo(visualization_image);

  // 在RGB图像上绘制检测到的边界框
  for (size_t i = 0; i < vllm_output.bounding_boxes_info.size(); ++i) {
    const auto& bbox_info = vllm_output.bounding_boxes_info[i];
    const cv::Rect& bbox = bbox_info.bounding_box;

    // 使用generateColor方法为每个边界框生成颜色
    Color color = generateColor(bbox_info.id);
    cv::Scalar cv_color(color.b, color.g, color.r);  // OpenCV使用BGR顺序

    // 绘制边界框
    cv::rectangle(visualization_image, bbox, cv_color, 2);

    // 准备标签文本
    std::string label = bbox_info.toString();

    // 计算标签尺寸并绘制标签背景
    int baseline = 0;
    cv::Size label_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    cv::Rect label_rect(bbox.x, bbox.y - label_size.height - baseline - 2,
                        label_size.width, label_size.height + baseline + 2);

    // 确保标签不会超出图像边界
    label_rect.x = std::max(
        0, std::min(label_rect.x, visualization_image.cols - label_rect.width));
    label_rect.y = std::max(0, std::min(label_rect.y, visualization_image.rows -
                                                          label_rect.height));

    // 绘制标签背景和文字
    cv::rectangle(visualization_image, label_rect, cv_color, -1);  // 填充矩形
    cv::putText(visualization_image, label,
                cv::Point(label_rect.x,
                          label_rect.y + label_size.height + baseline / 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
  }

  // 创建mask可视化图像
  cv::Mat mask_visualization;
  visualization_image.copyTo(mask_visualization);

  // 在图像上绘制mask
  std::unordered_map<int, Color> color_map;
  // 存储每个submap的中心点
  std::unordered_map<int, std::vector<cv::Point>> submap_centers;

  for (int y = 0; y < image_data->id_image_data.rows; ++y) {
    for (int x = 0; x < image_data->id_image_data.cols; ++x) {
      int submap_id = image_data->id_image_data.at<int>(y, x);
      if (submap_id >= 0) {  // 有效mask像素
        // 使用generateColor方法为每个submap ID生成颜色
        if (color_map.count(submap_id) == 0) {
          color_map[submap_id] = generateColor(submap_id);
        }
        Color& color = color_map[submap_id];

        // 在mask可视化图像上绘制半透明的mask
        cv::Vec3b& pixel = mask_visualization.at<cv::Vec3b>(y, x);
        pixel[0] =
            static_cast<unsigned char>(0.6 * pixel[0] + 0.4 * color.b);  // Blue
        pixel[1] = static_cast<unsigned char>(0.6 * pixel[1] +
                                              0.4 * color.g);  // Green
        pixel[2] =
            static_cast<unsigned char>(0.6 * pixel[2] + 0.4 * color.r);  // Red

        // 收集submap像素位置，用于计算中心点
        submap_centers[submap_id].push_back(cv::Point(x, y));
      }
    }
  }

  // 在每个submap的中心位置添加文本信息
  for (const auto& pair : submap_centers) {
    int submap_id = pair.first;

    // 计算submap的中心点
    long long sum_x = 0, sum_y = 0;
    const auto& points = pair.second;
    for (const auto& point : points) {
      sum_x += point.x;
      sum_y += point.y;
    }
    cv::Point center(sum_x / points.size(), sum_y / points.size());

    // 获取submap对应的物体类别
    std::string class_name = "Background";
    if (image_data->associated_submaps.find(submap_id) !=
        image_data->associated_submaps.end()) {
      class_name = image_data->associated_submaps.at(submap_id).class_name;
    }

    // 构造显示文本
    std::string text = std::to_string(submap_id) + ": " + class_name;

    // 设置文本参数
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.5;
    int thickness = 1;

    // 获取文本大小
    int baseline = 0;
    cv::Size text_size =
        cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

    // 调整文本位置，确保在图像范围内
    center.x = std::max(
        text_size.width / 2,
        std::min(center.x, mask_visualization.cols - text_size.width / 2));
    center.y = std::max(text_size.height,
                        std::min(center.y, mask_visualization.rows - baseline));

    // 绘制文本背景
    cv::Rect text_bg_rect(center.x - text_size.width / 2 - 2,
                          center.y - text_size.height - 2, text_size.width + 4,
                          text_size.height + baseline + 4);

    // 绘制半透明背景
    if (text_bg_rect.x >= 0 && text_bg_rect.y >= 0 &&
        text_bg_rect.x + text_bg_rect.width <= mask_visualization.cols &&
        text_bg_rect.y + text_bg_rect.height <= mask_visualization.rows) {
      cv::Mat roi = mask_visualization(text_bg_rect);
      cv::Mat color_bg(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
      cv::addWeighted(roi, 0.3, color_bg, 0.7, 0, roi);
    }

    // 绘制文本
    if (text_bg_rect.x >= 0 && text_bg_rect.y >= 0 &&
        text_bg_rect.x + text_bg_rect.width <= mask_visualization.cols &&
        text_bg_rect.y + text_bg_rect.height <= mask_visualization.rows) {
      cv::putText(mask_visualization, text,
                  cv::Point(center.x - text_size.width / 2, center.y),
                  font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
    }
  }

  // 保存可视化结果
  std::string dir = getVllmMiddleResultsDir();
  int image_id = image_data->image_id;
  std::string rgb_output_path =
      dir + "/" + std::to_string(image_id) + "_boxes.png";
  std::string mask_output_path =
      dir + "/" + std::to_string(image_id) + "_boxes_and_mask.png";

  cv::imwrite(rgb_output_path, visualization_image);
  cv::imwrite(mask_output_path, mask_visualization);

  LOG(INFO) << "Saved VLLM RGB visualization to: " << rgb_output_path;
  LOG(INFO) << "Saved VLLM mask visualization to: " << mask_output_path;

  // 将VLLMOutputData中的description信息输出到文本文件中
  std::string description_output_path =
      dir + "/" + std::to_string(image_id) + "_vllm_description.txt";
  std::ofstream description_file(description_output_path);
  if (description_file.is_open()) {
    description_file << "Bounding Boxes Information:\n";
    description_file << "==========================\n\n";

    for (size_t i = 0; i < vllm_output.bounding_boxes_info.size(); ++i) {
      const auto& bbox_info = vllm_output.bounding_boxes_info[i];
      const cv::Rect& bbox = bbox_info.bounding_box;
      description_file << "Box\n";
      description_file << "  ID: " << bbox_info.id << "\n";
      description_file << "  bbox: [" << bbox.x << ", " << bbox.y << ", "
                       << (bbox.x + bbox.width) << ", "
                       << (bbox.y + bbox.height) << "]\n";
      description_file << "  Description: " << bbox_info.description.toString()
                       << "\n";
      description_file << "\n";
    }

    for (const auto& re : vllm_output.bounding_boxes_relationships) {
      description_file << "Relationship\n";
      description_file << "  From: " << re.from_id << ",";
      description_file << "To: " << re.to_id << "\n";
      description_file << "  Relationship: "
                       << relationshipTypeToString(re.relationship) << "\n";
    }

    description_file << "\n\nBox and submap match result information:\n";
    description_file << "==========================\n\n";
    for (const auto& [box_id, submap_id] : box_submap_pair) {
      description_file << "Box ID: " << box_id
                       << " matched with Submap ID: " << submap_id << "\n";
    }

    description_file.close();
    LOG(INFO) << "Saved VLLM description to: " << description_output_path;
  } else {
    LOG(ERROR) << "Failed to open file for writing VLLM description: "
               << description_output_path;
  }
}

}  // namespace panoptic_mapping