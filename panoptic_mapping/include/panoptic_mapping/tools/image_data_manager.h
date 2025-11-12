#ifndef PANOPTIC_MAPPING_TOOLS_IMAGE_DATA_MANAGER_H_
#define PANOPTIC_MAPPING_TOOLS_IMAGE_DATA_MANAGER_H_

#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/common/input_data.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

// 存储单个图像的信息
struct ImageData {
  int image_id;                     // 图像唯一ID
  double timestamp;                 // 图像时间戳
  std::string rbg_image_file_name;  // RGB图像文件名（具体路径由其他参数给定）
  std::string id_image_file_name;   // ID图像文件名
  std::unordered_set<int> associated_submaps;  // 关联的submap IDs
  bool is_processed = false;                   // 是否已被VL大模型处理

  // 内存中的实际图像数据（仅在缓存中存在）
  cv::Mat rgb_data;       // RGB图像数据（内存中）
  cv::Mat id_image_data;  // ID图像数据（内存中）

  // 默认构造函数
  ImageData() = default;

  // 拷贝构造函数
  ImageData(const ImageData& other);

  // 拷贝赋值运算符
  ImageData& operator=(const ImageData& other);

  // 移动构造函数
  ImageData(ImageData&& other) = default;

  // 移动赋值运算符
  ImageData& operator=(ImageData&& other) = default;
};

struct BoundingBoxInfoByVLLM {
  // 返回的 box 的唯一标识 id
  int id;
  // box 框
  cv::Rect bounding_box;
  // 自身描述
  std::vector<std::string> description;
};

struct VLLMOutputData {
  int image_id;  // 图像ID
  std::vector<BoundingBoxInfoByVLLM>
      bounding_boxes_info;  // VLLM返回的bounding box信息
};

/**
 * @brief ImageDataManager 处理的主体是 rgb 图像以及 mask 图像，mask
 * 图像的像素是 submap ID
 *
 */
class ImageDataManager {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;

    // 图像保存路径
    std::string image_save_directory = "/tmp/panoptic_mapping_images";
    // 图像管理数据的文件名，这里保存的就是图片地图信息。该文件需要与图像文件在同一文件夹下
    std::string meta_infos_file_name = "images_meta_infos.bin";

    // 内存中最多保存的图像数量
    int max_images_in_memory = 10;

    // 是否启用图像管理
    bool enable_image_management = true;

    // 是否读取历史保存的信息（作为地图的一部分）
    bool load_image_data_info_on_startup = false;

    Config() { setConfigName("ImageDataManager"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit ImageDataManager(const Config& config);
  virtual ~ImageDataManager() = default;

  // 添加新的图像数据，直接持久化到硬盘
  int addImageData(const cv::Mat& image, const cv::Mat& id_image,
                   const DetectronLabels& detectron_labels, double timestamp,
                   const SubmapCollection& submaps);

  // 获取第一个未处理图像数据，用于VL大模型输入
  std::shared_ptr<ImageData> getFirstNotProcessedImageDataForVLLM();

  // 处理VL大模型输出
  void processVLLMOutput(const VLLMOutputData& vllm_output,
                         SubmapCollection& submaps);

  // 根据submap ID获取所有关联的图像
  std::shared_ptr<ImageData> getImageForSubmap(int submap_id);

  // 序列化图像信息到文件
  void saveMappingsToFile(const std::string& filepath) const;

  int unprocessedImageDataSize() const { return unprocessed_images_.size(); }

 private:
  // 从文件加载图像信息
  void loadMappingsFromFile(const std::string& filepath);

  // 根据 VL 大模型返回的数据，更新 submap 描述信息
  void updateSubmap(BoundingBoxInfoByVLLM box_info, Submap* submap);

  // 标记图像已被处理
  void markImageAsProcessed(int image_id);

  // 建立submap和图像之间的关联
  void associateSubmapWithImage(int submap_id, int image_id);

  // 解除submap和图像之间的关联
  void dissociateSubmapFromImage(int submap_id, int image_id);
  // 当submap被删除时，清理相关联的图像数据
  void handleSubmapRemoval(int submap_id);

  // 清理不关联任何submap的图像数据
  void cleanupUnassociatedImages();
  // 获取图像数据（仅元数据）
  std::shared_ptr<ImageData> getImageData(int image_id);
  // 判断是否需要保留图像
  bool needToRetainImageData(const SubmapCollection& submaps);

  // 获取被删除的 submaps
  void getDeletedSubmaps(const SubmapCollection& submaps,
                         std::vector<int>& deleted_submap_ids);

  // 保存图像到磁盘
  bool saveImageToDisk(const cv::Mat& image, const cv::Mat& id_image,
                       const std::string& rgb_image_name,
                       const std::string& id_image_name);

  // 从磁盘加载图像
  bool loadImageFromDisk(const std::shared_ptr<ImageData>& image_data);

  // 删除磁盘上的图像文件
  bool deleteImageFromDisk(const std::shared_ptr<ImageData>& image_data);

  // 自动关联图像中的submap ID
  void autoAssociateSubmaps(int image_id, const cv::Mat& id_image,
                            const SubmapCollection& submaps);

  // 初始化ID计数器，从已保存的图像中恢复最大ID
  void initializeImageIdCounter();

  // 从文件名中提取ID
  int extractIdFromFilename(const std::string& filename) const;

  // 管理内存缓存
  void addToCache(const std::shared_ptr<ImageData>& image_data);
  void removeFromCache(int image_id);
  void updateCacheAccess(int image_id);
  void evictCacheIfNeeded();

  // 根据文件名获取图像路径
  std::string getImagePath(const std::string& file_name) const;
  // 图像管理信息的文件路径
  std::string getImageDataInfoPath() const;

  // 创建仅包含元数据的图像数据副本
  std::shared_ptr<ImageData> createMetadataCopy(
      const std::shared_ptr<ImageData>& source);

 private:
  const Config config_;

  // 图像数据存储（仅元数据，无实际图像数据）
  std::unordered_map<int, std::shared_ptr<ImageData>> image_data_;

  // Submap到图像的映射（一对多）
  std::unordered_map<int, std::unordered_set<int>> submap_to_images_;

  // 图像到Submap的映射（一对多）
  std::unordered_map<int, std::unordered_set<int>> image_to_submaps_;

  // 内存中的实际图像数据缓存（用于提高访问效率）
  std::unordered_map<int, std::shared_ptr<ImageData>> image_cache_;

  // 未处理图像ID集合
  std::unordered_set<int> unprocessed_images_;

  // 图像访问历史（用于LRU缓存策略）
  std::list<int> image_access_history_;

  // 当前图像ID计数器
  int current_image_id_ = 0;

  // 互斥锁保护线程安全
  mutable std::mutex mutex_;

  // 存储上一轮的 submap id，用于检测变化
  std::set<int> last_submap_ids_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_IMAGE_DATA_MANAGER_H_