#include "panoptic_mapping/map_management/change_detector.h"

#include <future>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {
void ChangeDetector::Config::checkParams() const {
  checkParamNE(strong_disappear_threshold, 0.f, "strong_disappear_threshold");
  checkParamNE(weak_disappear_threshold, 0.f, "weak_disappear_threshold");
  checkParamGE(match_strong_disappear_points, 0,
               "match_strong_disappear_points");
  checkParamGE(match_weak_disappear_points, 0, "match_weak_disappear_points");
  checkParamGE(match_strong_disappear_percentage, 0.f,
               "match_strong_disappear_percentage");
  checkParamLE(match_strong_disappear_percentage, 1.f,
               "match_strong_disappear_percentage");
  checkParamGE(match_weak_disappear_percentage, 0.f,
               "match_weak_disappear_percentage");
  checkParamLE(match_weak_disappear_percentage, 1.f,
               "match_weak_disappear_percentage");
  checkParamNE(match_weak_average_distance, 0.f, "match_weak_average_distance");
  checkParamGT(detection_threads, 0, "detection_threads");
}

void ChangeDetector::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("strong_disappear_threshold", &strong_disappear_threshold);
  setupParam("weak_disappear_threshold", &weak_disappear_threshold);
  setupParam("match_strong_disappear_points", &match_strong_disappear_points);
  setupParam("match_weak_disappear_points", &match_weak_disappear_points);
  setupParam("match_strong_disappear_percentage",
             &match_strong_disappear_percentage);
  setupParam("match_weak_disappear_percentage",
             &match_weak_disappear_percentage);
  setupParam("match_weak_average_distance", &match_weak_average_distance);
  setupParam("detection_threads", &detection_threads);
  setupParam("use_classification_for_tiny", &use_classification_for_tiny);
  setupParam("classification_disappear_threshold",
             &classification_disappear_threshold);
  setupParam("classification_disappear_percentage",
             &classification_disappear_percentage);
  setupParam("classification_disappear_average_distance",
             &classification_disappear_average_distance);
  setupParam("classification_projected_percentage",
             &classification_projected_percentage);
  setupParam("classification_only_background", &classification_only_background);
  setupParam("limit_range", &limit_range);
  setupParam("classification_disappear_frames_threshold",
             &classification_disappear_frames_threshold);
  setupParam("classification_use_no_class", &classification_use_no_class);
  setupParam("min_isolated_points_size", &min_isolated_points_size);
}

ChangeDetector::ChangeDetector(const Config& config,
                               std::shared_ptr<Globals> globals)
    : config_(config.checkValid()), globals_(std::move(globals)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void ChangeDetector::checkSubmapCollectionVisibleByInputData(
    SubmapCollection* submaps, InputData* input) {
  auto t_start = std::chrono::high_resolution_clock::now();
  std::string info;

  // Check all inactive maps for absent detect
  std::vector<int> id_list;
  const Camera& camera = *globals_->camera();
  const Transformation& T_M_C = input->T_M_C();

  const std::unordered_set<std::string> whitelist = globals_->getWhiteList();
  for (const Submap& submap : *submaps) {
    if (!submap.isActive() && submap.getLabel() != PanopticLabel::kFreeSpace &&
        !submap.getIsoSurfacePoints().empty() &&
        submap.getChangeState() != ChangeState::kAbsent) {
      const Point center_C = T_M_C.inverse() * submap.getT_M_S() *
                             submap.getBoundingVolume().getCenter();
      if (!camera.submapIsInViewFrustum(submap, T_M_C)) {
        continue;
      }

      if (!whitelist.empty() && whitelist.count(submap.getClassName()) == 0) {
        continue;
      }
      id_list.emplace_back(submap.getID());
    }
  }

  if (id_list.empty()) {
    return;
  }

  if (config_.verbosity >= 2) {
    std::stringstream ss;
    for (const int id : id_list) {
      ss << id << "(" << submaps->getSubmapPtr(id)->getClassName() << "),";
    }
    LOG(INFO) << "Submaps to check change by input data:" << ss.str();
  }

  // Perform change detection in parallel.
  SubmapIndexGetter index_getter(id_list);
  std::vector<std::future<std::string>> threads;
  for (int i = 0; i < config_.detection_threads; ++i) {
    threads.emplace_back(
        std::async(std::launch::async, [this, &index_getter, submaps, input]() {
          int index;
          std::string info;
          while (index_getter.getNextIndex(&index)) {
            info += this->checkSubmapVisibleByInputData(
                submaps->getSubmapPtr(index), input);
            if (config_.use_classification_for_tiny) {
              info += this->checkSubmapVisibleByInputDataWithClassification(
                  submaps->getSubmapPtr(index), input);
            }
          }
          return info;
        }));
  }

  // Join all threads.
  for (auto& thread : threads) {
    info += thread.get();
  }
  auto t_end = std::chrono::high_resolution_clock::now();

  LOG_IF(INFO, config_.verbosity >= 2)
      << "Performed change detection by input data in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
             .count()
      << (config_.verbosity < 3 || info.empty() ? "ms." : "ms:" + info);
}

std::string ChangeDetector::checkSubmapVisibleByInputData(Submap* submap,
                                                          InputData* input) {
  auto T_C_S = input->T_M_C().inverse() * submap->getT_M_S();
  const Camera& camera = *globals_->camera();
  const cv::Mat& depth_image = input->depthImage();
  const cv::Mat& vertex_image = input->vertexMap();

  int strong_absent_num = 0;
  int weak_absent_num = 0;
  float weak_absent_dis_sum = 0.0;
  int valid_depth_measurement_num = 0;
  int projected_num = 0;

  float strong_depth_tolerance = config_.strong_disappear_threshold > 0
                                     ? config_.strong_disappear_threshold
                                     : -config_.strong_disappear_threshold *
                                           submap->getTsdfLayer().voxel_size();
  float weak_depth_tolerance = config_.weak_disappear_threshold > 0
                                   ? config_.weak_disappear_threshold
                                   : -config_.weak_disappear_threshold *
                                         submap->getTsdfLayer().voxel_size();

  // Simply limit the measurement values of the depth measurement
  float camera_visible_distance_max = 5.0 * camera.getConfig().max_range;

  float camera_min_range = camera.getConfig().min_range;
  float camera_max_range = camera.getConfig().max_range;

  for (const auto& point : submap->getIsoSurfacePoints()) {
    const auto p_C = T_C_S * point.position;
    int u, v;
    if (!camera.projectPointToImagePlane(p_C, &u, &v)) {
      continue;
    }

    projected_num++;
    float depth_value = depth_image.at<float>(v, u);

    if (config_.limit_range) {
      if (depth_value < camera_min_range || depth_value > camera_max_range) {
        continue;
      }
    }

    if (depth_value != 0.f) {
      valid_depth_measurement_num++;
    }

    const cv::Vec3f& vertex = vertex_image.at<cv::Vec3f>(v, u);
    float vertex_range_dis = std::sqrt(
        vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]);
    float p_C_dis =
        std::sqrt(p_C.x() * p_C.x() + p_C.y() * p_C.y() + p_C.z() * p_C.z());
    float distance = vertex_range_dis - p_C_dis;
    distance = std::min(distance, camera_visible_distance_max);

    if (distance >= strong_depth_tolerance) {
      strong_absent_num++;
      weak_absent_num++;
      weak_absent_dis_sum += distance;
    } else if (distance >= weak_depth_tolerance) {
      weak_absent_num++;
      weak_absent_dis_sum += distance;
    }
  }

  int strong_disappear_num_threshold =
      std::max(config_.match_strong_disappear_points,
               static_cast<int>(config_.match_strong_disappear_percentage *
                                submap->getIsoSurfacePoints().size()));

  int weak_disappear_num_threshold =
      std::max(config_.match_weak_disappear_points,
               static_cast<int>(config_.match_weak_disappear_percentage *
                                submap->getIsoSurfacePoints().size()));
  if (strong_absent_num > strong_disappear_num_threshold) {
    submap->setChangeState(ChangeState::kAbsent);
    std::stringstream info;
    info << "\nSubmap " << submap->getID() << " (" << submap->getName()
         << ") conflicts with input data judged by strong. Marked as absent.";
    return info.str();
  }

  if (weak_absent_num > weak_disappear_num_threshold) {
    float weak_avg_dis = weak_absent_dis_sum / weak_absent_num;
    float weak_avg_dis_threshold =
        config_.match_weak_average_distance > 0
            ? config_.match_weak_average_distance
            : -config_.match_weak_average_distance *
                  submap->getTsdfLayer().voxel_size();
    if (weak_avg_dis >= weak_avg_dis_threshold) {
      submap->setChangeState(ChangeState::kAbsent);
      std::stringstream info;
      info << "\nSubmap " << submap->getID() << " (" << submap->getName()
           << ") conflicts with input data judged by weak. Marked as absent."
           << " Absent points: (" << strong_absent_num << "," << weak_absent_num
           << ")"
           << "/" << submap->getIsoSurfacePoints().size()
           << ", weak distance sum: " << weak_absent_dis_sum << " m. "
           << "valid_measurement_nums / projected_nums: "
           << valid_depth_measurement_num << " / " << projected_num;
      return info.str();
    }
  }

  std::stringstream info;
  info << "\nSubmap " << submap->getID() << " (" << submap->getName()
       << ") is valid with input data. Absent points: (" << strong_absent_num
       << "," << weak_absent_num << ")"
       << "/" << submap->getIsoSurfacePoints().size()
       << ", weak distance sum: " << weak_absent_dis_sum << " m. "
       << "valid_measurement_nums / projected_nums: "
       << valid_depth_measurement_num << " / " << projected_num;
  return info.str();
}

std::string ChangeDetector::checkSubmapVisibleByInputDataWithClassification(
    Submap* submap, InputData* input) {
  if (submap->getIsoSurfacePoints().size() > config_.min_isolated_points_size) {
    std::stringstream info;
    info << "\nSubmap " << submap->getID() << " (" << submap->getName()
         << ") points size: " << submap->getIsoSurfacePoints().size() << " > "
         << config_.min_isolated_points_size;
    return info.str();
  }

  auto T_C_S = input->T_M_C().inverse() * submap->getT_M_S();
  const Camera& camera = *globals_->camera();
  const cv::Mat& depth_image = input->depthImage();
  const cv::Mat& id_image_copy = input->idImageCopy();
  const cv::Mat& vertex_image = input->vertexMap();

  int absent_num = 0;
  float absent_dis_sum = 0.0;
  int valid_depth_measurement_num = 0;
  int projected_num = 0;

  float depth_tolerance = config_.classification_disappear_threshold > 0
                              ? config_.classification_disappear_threshold
                              : -config_.classification_disappear_threshold *
                                    submap->getTsdfLayer().voxel_size();

  // Simply limit the measurement values of the depth measurement
  float camera_visible_distance_max = 5.0 * camera.getConfig().max_range;

  float camera_min_range = camera.getConfig().min_range;
  float camera_max_range = camera.getConfig().max_range;

  std::unordered_map<int, int> projected_instance_nums;
  for (const auto& point : submap->getIsoSurfacePoints()) {
    const auto p_C = T_C_S * point.position;
    int u, v;
    if (!camera.projectPointToImagePlane(p_C, &u, &v)) {
      continue;
    }

    projected_num++;
    float depth_value = depth_image.at<float>(v, u);

    if (config_.limit_range) {
      if (depth_value < camera_min_range || depth_value > camera_max_range) {
        continue;
      }
    }

    if (depth_value != 0.f) {
      valid_depth_measurement_num++;
    }

    const cv::Vec3f& vertex = vertex_image.at<cv::Vec3f>(v, u);
    float vertex_range_dis = std::sqrt(
        vertex[0] * vertex[0] + vertex[1] * vertex[1] + vertex[2] * vertex[2]);
    float p_C_dis =
        std::sqrt(p_C.x() * p_C.x() + p_C.y() * p_C.y() + p_C.z() * p_C.z());
    float distance = vertex_range_dis - p_C_dis;
    distance = std::min(distance, camera_visible_distance_max);

    if (distance >= depth_tolerance) {
      absent_num++;
      absent_dis_sum += distance;
    }

    int instance_id = id_image_copy.at<int>(v, u);
    projected_instance_nums[instance_id]++;
  }

  int max_instance_id = -1;
  int max_projected_num = -1;
  for (auto& pair : projected_instance_nums) {
    if (pair.second > max_projected_num) {
      max_projected_num = pair.second;
      max_instance_id = pair.first;
    }
  }

  if (config_.classification_use_no_class) {
    if (projected_instance_nums.count(0) != 0) {
      max_projected_num += projected_instance_nums[0];
    }
  }

  const DetectronLabels* labels = &(input->detectronLabels());
  std::string info_str;
  std::string background_class_name;
  if (!validWithClassification(max_instance_id, submap, labels, info_str,
                               background_class_name)) {
    return info_str;
  }

  int disappear_num_threshold =
      static_cast<int>(config_.classification_disappear_percentage *
                       submap->getIsoSurfacePoints().size());

  int min_projected_other_type_num =
      static_cast<int>(config_.classification_projected_percentage *
                       submap->getIsoSurfacePoints().size());

  float avg_dis_threshold =
      config_.classification_disappear_average_distance > 0
          ? config_.classification_disappear_average_distance
          : -config_.classification_disappear_average_distance *
                submap->getTsdfLayer().voxel_size();

  bool disappear = false;
  if (max_projected_num > min_projected_other_type_num &&
      absent_num > disappear_num_threshold) {
    float absent_avg_distance = absent_dis_sum / absent_num;
    if (absent_avg_distance > avg_dis_threshold) {
      disappear = true;
    }
  }

  if (disappear) {
    submap->addDisappearCount();
    if (submap->getDisappearCount() >
        config_.classification_disappear_frames_threshold) {
      submap->setChangeState(ChangeState::kAbsent);
      std::stringstream info;
      info << "\nSubmap " << submap->getID() << " (" << submap->getName()
           << ") conflicts with input data judged by classification. ("
           << background_class_name << ") "
           << " Marked as absent." << " Absent points: " << absent_num << "/"
           << submap->getIsoSurfacePoints().size()
           << ", distance sum: " << absent_dis_sum << " m. "
           << "projected_on (" << background_class_name
           << ") num: " << max_projected_num
           << ", valid_measurement_nums / projected_nums: "
           << valid_depth_measurement_num << " / " << projected_num;
      return info.str();
    }
  }

  std::stringstream info;
  info << "\nSubmap " << submap->getID() << " (" << submap->getName()
       << ") is valid with input data by classification. Absent points: "
       << absent_num << "/" << submap->getIsoSurfacePoints().size()
       << ", distance sum: " << absent_dis_sum << " m. "
       << "projected_on (" << background_class_name
       << ") num: " << max_projected_num
       << ", valid_measurement_nums / projected_nums: "
       << valid_depth_measurement_num << " / " << projected_num
       << ", current frame disappear status(1: disappear): " << disappear
       << ", disappear frame: " << submap->getDisappearCount();
  return info.str();
}

bool ChangeDetector::validWithClassification(
    int projected_instance_id, Submap* submap, const DetectronLabels* labels,
    std::string& info_str, std::string& background_class_name) {
  if (projected_instance_id < 0) {
    std::stringstream info;
    info << "\nSubmap " << submap->getID() << " (" << submap->getName()
         << ") not project on detected instance(instance_id: "
         << projected_instance_id << ").";
    info_str = info.str();
    return false;
  }

  if (config_.classification_use_no_class) {
    if (projected_instance_id == 0) {
      std::stringstream info;
      info << "\nSubmap " << submap->getID() << " (" << submap->getName()
           << ") project on non class area(instance_id: "
           << projected_instance_id << ").";
      info_str = info.str();
      background_class_name = "no_class";
      return true;
    }
  }

  if (projected_instance_id == 0) {
    std::stringstream info;
    info << "\nSubmap " << submap->getID() << " (" << submap->getName()
         << ") not project on detected instance(instance_id: "
         << projected_instance_id << ").";
    info_str = info.str();
    return false;
  }

  auto it = labels->find(projected_instance_id);
  if (it == labels->end()) {
    std::stringstream info;
    info << "\nSubmap " << submap->getID() << " (" << submap->getName()
         << ") not project on detected instance(instance_id: "
         << projected_instance_id << " not found).";
    info_str = info.str();
    return false;
  }

  // Find background to judge
  if (config_.classification_only_background && it->second.is_thing) {
    std::stringstream info;
    info << "\nSubmap " << submap->getID() << " (" << submap->getName()
         << ") not project on detected instance(instance_id: "
         << projected_instance_id << " is a thing not background).";
    info_str = info.str();
    return false;
  }

  background_class_name = it->second.category_name;
  if (background_class_name == submap->getClassName()) {
    info_str = "";
    return false;
  }

  return true;
}

}  // namespace panoptic_mapping