#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_CHANGE_DETECTOR_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_CHANGE_DETECTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/globals.h"
#include "panoptic_mapping/common/input_data.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

class ChangeDetector {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Limit range in valid depth to judge
    bool limit_range = true;

    // Only judge submap center in (camera.max_range - range_inner_buffer)
    float range_inner_buffer = 0.0;

    // Allowed disappear distance in meters where a point is still considered
    // visible in input data. Negative values are multiples of the voxel_size.
    float strong_disappear_threshold = -10;
    float weak_disappear_threshold = -2;

    // Minimum number of points required for a submap to considered disappear.
    int match_strong_disappear_points = 50;
    int match_weak_disappear_points = 50;

    // Minimum percentage of points required for a submap to considered
    // disappear.
    float match_strong_disappear_percentage = 0.6;
    float match_weak_disappear_percentage = 0.8;

    // For weak disappear point, the average distance from disappear points to
    // the projected pixel depth must larger than this value. Unit m, Negative
    // values are multiples of the voxel_size. This must larger than
    // weak_disappear_threshold
    float match_weak_average_distance = -3;

    // Number of threads used to perform change detection. Change detection is
    // submap-parallel.
    int detection_threads = std::thread::hardware_concurrency();

    // Whether to use the classification information in input data to detect
    // change for tiny object
    bool use_classification_for_tiny = false;
    int min_isolated_points_size = 500;
    // Only handle tiny objects on background
    bool classification_only_background = true;
    // Use no class as a type of class
    bool classification_use_no_class = false;
    // Allowed disappear distance in meters where a point is still considered
    // visible in input data. Negative values are multiples of the voxel_size.
    float classification_disappear_threshold = -1;
    // Minimum percentage of points required for a submap to considered
    // disappear.
    float classification_disappear_percentage = 0.5;
    float classification_disappear_average_distance = -1;
    // Min percentage of points belong to other type.
    float classification_projected_percentage = 0.9;

    // For classification.
    int classification_disappear_frames_threshold = 3;

    // Judge whether the camera motion is soft, unit: meter/s and deg/s
    float max_translation_velocity = 0.5;
    float max_rotation_velocity = 5.0;

    Config() { setConfigName("ChangeDetector"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  ChangeDetector(const Config& config, std::shared_ptr<Globals> globals);
  virtual ~ChangeDetector() = default;

  // Check submap that in current view frustum but not visible
  // on input image, this submap is absent
  void checkSubmapCollectionVisibleByInputData(SubmapCollection* submaps,
                                               InputData* input);

 private:
  std::string checkSubmapVisibleByInputData(Submap* submap, InputData* input);
  std::string checkSubmapVisibleByInputDataWithClassification(Submap* submap,
                                                              InputData* input);
  bool validWithClassification(int projected_instance_id, Submap* submap,
                               const DetectronLabels* labels, std::string& info,
                               std::string& background_class_name);
                              //  判断相机运动是否较小
                          bool cameraMotionSoft(const Transformation& T_M_C,
                                               double timestamp);

  const Config config_;
  const std::shared_ptr<Globals> globals_;

 private:
  // 记录上一帧数据的位姿以及时间戳，用来判断相机运动程度
  std::shared_ptr<Transformation> last_camera_pose_ = nullptr;
  double last_camera_timestamp_ = -1;
};

}  // namespace panoptic_mapping

#endif