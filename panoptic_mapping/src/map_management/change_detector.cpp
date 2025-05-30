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
  for (const Submap& submap : *submaps) {
    if (!submap.isActive() && submap.getLabel() != PanopticLabel::kFreeSpace &&
        !submap.getIsoSurfacePoints().empty() &&
        submap.getChangeState() != ChangeState::kAbsent) {
      const Point center_C = T_M_C.inverse() * submap.getT_M_S() *
                             submap.getBoundingVolume().getCenter();
      if (!camera.pointIsInViewFrustum(center_C)) {
        continue;
      }
      id_list.emplace_back(submap.getID());
    }
  }

  if (id_list.empty()) {
    return;
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

  int strong_absent_num = 0;
  int weak_absent_num = 0;
  float weak_absent_dis_sum = 0.0;

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

  for (const auto& point : submap->getIsoSurfacePoints()) {
    const auto p_C = T_C_S * point.position;
    int u, v;
    if (!camera.projectPointToImagePlane(p_C, &u, &v)) {
      continue;
    }
    float distance = depth_image.at<float>(v, u) - p_C.z();
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
           << ") conflicts with input data judged by weak. Marked as absent.";
      return info.str();
    }
  }

  std::stringstream info;
  info << "\nSubmap " << submap->getID() << " (" << submap->getName()
       << ") is valid with input data. Absent points: (" << strong_absent_num
       << "," << weak_absent_num << ")"
       << "/" << submap->getIsoSurfacePoints().size()
       << ", weak distance sum: " << weak_absent_dis_sum << " m.";
  return info.str();
}

}  // namespace panoptic_mapping