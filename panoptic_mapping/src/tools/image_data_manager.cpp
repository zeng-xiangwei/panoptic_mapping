#include "panoptic_mapping/tools/image_data_manager.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>

#include <opencv2/imgcodecs.hpp>

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

    // 初始化ID计数器
    if (config_.load_image_data_info_on_startup) {
      std::string meta_file_path = getImageDataInfoPath();
      LOG(INFO) << "Loading image data info from " << meta_file_path;
      loadMappingsFromFile(meta_file_path);
    }
    initializeImageIdCounter();
  }
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

  // 创建包含实际图像数据的缓存副本
  auto cache_data = std::make_shared<ImageData>(*image_data);
  cache_data->rgb_data = image.clone();
  cache_data->id_image_data = id_image.clone();
  addToCache(cache_data);

  // 自动关联图像中的submap
  autoAssociateSubmaps(image_data->image_id, id_image, submaps);
  unprocessed_images_.insert(image_data->image_id);

  LOG_IF(INFO, config_.verbosity >= 2)
      << "Added image data with ID " << image_data->image_id
      << " and associated with " << image_data->associated_submaps.size()
      << " submaps.";

  // 处理已经删除的 submap
  std::vector<int> deleted_submap_ids;
  getDeletedSubmaps(submaps, deleted_submap_ids);
  for (int submap_id : deleted_submap_ids) {
    handleSubmapRemoval(submap_id);
  }

  return image_data->image_id;
}

bool ImageDataManager::needToRetainImageData(const SubmapCollection& submaps) {
  std::unordered_set<int> activate_submap_ids;
  for (const Submap& submap : submaps) {
    if (submap.isActive() && submap.matchRedetection()) {
      // 满足重复观测条件的活跃 submap
      activate_submap_ids.insert(submap.getID());
    }
  }

  for (auto k : activate_submap_ids) {
    if (submap_to_images_.count(k) == 0) {
      LOG(INFO) << "Submap " << k
                << " is active but not in the image data manager, this "
                   "ImageData will be retained.";
      return true;
    }
  }

  return false;
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
  int image_id = *unprocessed_images_.begin();

  // 获取图像数据
  auto image_data = getImageData(image_id);
  if (!image_data) {
    // 如果获取失败，从未处理集合中移除
    unprocessed_images_.erase(image_id);
    return nullptr;
  }

  unprocessed_images_.erase(image_id);

  return image_data;
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

  // TODO: 根据bounding_boxes为相关submap添加描述信息
  // 这里需要根据具体需求实现，例如:
  // 1.
  // 将bounding_boxes与id_image中对应的部分做关联，id_image中每个像素的值对应一个submap
  // ID，可以使用IoU的方式来匹配，这样就将 VLLM 输出的box与 submap
  // 关联到一起了。
  // 2. 为每个submap添加语义描述信息
  // 3. 更新submap的相关属性
  // 为每个边界框找到最佳匹配的submap
  for (const auto& bbox_info : vllm_output.bounding_boxes_info) {
    const cv::Rect& bbox = bbox_info.bounding_box;

    // 计算每个关联submap与边界框的IoU
    std::unordered_map<int, float> submap_ious;
    for (int submap_id : image_data->associated_submaps) {
      // 计算submap在ID图像中的覆盖区域
      int submap_pixel_count = 0;
      int intersection_count = 0;

      // 遍历边界框区域内的所有像素
      for (int y = std::max(0, bbox.y);
           y < std::min(bbox.y + bbox.height, image_data->id_image_data.rows);
           ++y) {
        for (int x = std::max(0, bbox.x);
             x < std::min(bbox.x + bbox.width, image_data->id_image_data.cols);
             ++x) {
          int pixel_submap_id = image_data->id_image_data.at<int>(y, x);
          if (pixel_submap_id == submap_id) {
            intersection_count++;
            submap_pixel_count++;
          } else if (pixel_submap_id >= 0) {
            submap_pixel_count++;
          }
        }
      }

      // 计算IoU
      if (submap_pixel_count > 0) {
        int bbox_area = bbox.width * bbox.height;
        float iou = static_cast<float>(intersection_count) /
                    (submap_pixel_count + bbox_area - intersection_count);
        submap_ious[submap_id] = iou;
      }
    }

    // 找到具有最高IoU的submap
    if (!submap_ious.empty()) {
      int best_submap_id = -1;
      float max_iou = -1.0f;

      for (const auto& pair : submap_ious) {
        if (pair.second > max_iou) {
          max_iou = pair.second;
          best_submap_id = pair.first;
        }
      }

      // 如果找到了匹配的submap且IoU足够高，则更新submap信息
      if (best_submap_id != -1 && max_iou > 0.1f) {  // IoU阈值可根据需要调整
        LOG(INFO) << "Associating bounding box " << bbox_info.id
                  << " with submap " << best_submap_id << " (IoU: " << max_iou
                  << ")";

        updateSubmap(bbox_info, submaps.getSubmapPtr(best_submap_id));
      }
    }
  }

  LOG(INFO) << "Processed VLLM output for image ID: " << vllm_output.image_id
            << " with " << vllm_output.bounding_boxes_info.size()
            << " bounding boxes";
}

void ImageDataManager::updateSubmap(BoundingBoxInfoByVLLM box_info,
                                    Submap* submap) {
  submap->setDescriptsByVllm(box_info.description);
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

void ImageDataManager::associateSubmapWithImage(int submap_id, int image_id) {
  if (!config_.enable_image_management) {
    return;
  }

  // 建立双向关联
  submap_to_images_[submap_id].insert(image_id);
  image_to_submaps_[image_id].insert(submap_id);

  // 在图像数据中也记录关联
  auto it = image_data_.find(image_id);
  if (it != image_data_.end()) {
    it->second->associated_submaps.insert(submap_id);
  }

  LOG_IF(INFO, config_.verbosity >= 3)
      << "Associated submap " << submap_id << " with image " << image_id;
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

  // TODO: 返回时间戳最新的数据
  std::vector<std::shared_ptr<ImageData>> result;
  auto it = submap_to_images_.find(submap_id);
  if (it != submap_to_images_.end()) {
    result.reserve(it->second.size());
    for (const auto& image_id : it->second) {
      const auto image_data = getImageData(image_id);
      if (image_data) {
        result.push_back(image_data);
      }
    }
  }

  if (result.empty()) {
    return nullptr;
  }
  return result[0];
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

  // 关联所有在当前submap集合中存在的ID
  for (int submap_id : unique_ids) {
    if (submaps.submapIdExists(submap_id)) {
      const Submap& submap = submaps.getSubmap(submap_id);
      associateSubmapWithImage(submap_id, image_id);
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

    // 写入关联的submap数量和ID列表
    uint32_t associated_submaps_count = image_data->associated_submaps.size();
    file.write(reinterpret_cast<const char*>(&associated_submaps_count),
               sizeof(associated_submaps_count));

    for (int submap_id : image_data->associated_submaps) {
      file.write(reinterpret_cast<const char*>(&submap_id), sizeof(submap_id));
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
    const std::unordered_set<int>& image_ids = pair.second;

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

    // 读取关联的submap ID列表
    uint32_t associated_submaps_count;
    file.read(reinterpret_cast<char*>(&associated_submaps_count),
              sizeof(associated_submaps_count));

    for (uint32_t j = 0; j < associated_submaps_count; ++j) {
      int submap_id;
      file.read(reinterpret_cast<char*>(&submap_id), sizeof(submap_id));
      image_data->associated_submaps.insert(submap_id);
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

}  // namespace panoptic_mapping