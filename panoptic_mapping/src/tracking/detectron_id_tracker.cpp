#include "panoptic_mapping/tracking/detectron_id_tracker.h"

#include <memory>
#include <unordered_map>
#include <utility>

#include "panoptic_mapping/common/index_getter.h"
#include "panoptic_mapping/map/class_name_manager.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, DetectronIDTracker,
                                           std::shared_ptr<Globals>>
    DetectronIDTracker::registration_("detectron");

void DetectronIDTracker::Config::checkParams() const {
  checkParamConfig(projective_id_tracker);
}

void DetectronIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("projective_id_tracker", &projective_id_tracker);
}

DetectronIDTracker::DetectronIDTracker(const Config& config,
                                       std::shared_ptr<Globals> globals)
    : config_(config.checkValid()),
      ProjectiveIDTracker(config.projective_id_tracker, std::move(globals),
                          false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  addRequiredInput(InputData::InputType::kDetectronLabels);
  whitelist_classes_ = globals_->getWhiteList();
  blacklist_classes_ = globals_->getBlackList();

  std::stringstream info;
  for (auto class_name : whitelist_classes_) {
    info << class_name << ",";
  }
  LOG_IF(INFO, config_.verbosity >= 1) << "Whitelist: " << info.str();

  info.clear();
  for (auto class_name : blacklist_classes_) {
    info << class_name << ",";
  }
  LOG_IF(INFO, config_.verbosity >= 1) << "Blacklist: " << info.str();
}

void DetectronIDTracker::processInput(SubmapCollection* submaps,
                                      InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));
  // Cache the input labels for submap allocation.
  labels_ = &(input->detectronLabels());

  // Track the predicted ids.
  ProjectiveIDTracker::processInput(submaps, input);
}

Submap* DetectronIDTracker::allocateSubmap(int input_id,
                                           SubmapCollection* submaps,
                                           InputData* input) {
  if (input_id == 0) {
    // The id 0 is used for no-predictions in detectron.
    return nullptr;
  }

  // Check whether the instance code is known.
  auto it = labels_->find(input_id);
  if (it == labels_->end()) {
    return nullptr;
  }

  if (!whitelist_classes_.empty() &&
      (whitelist_classes_.count(it->second.category_name) == 0)) {
    // Class is not in whitelist.
    return nullptr;
  }

  if (blacklist_classes_.count(it->second.category_name) != 0) {
    // 黑名单内的类别不构造
    return nullptr;
  }

  // Parse detectron label.
  LabelEntry label;
  label.name = it->second.category_name;
  const int class_id = ClassNameManager::getGlobalInstance()->getClassID(label.name);;
  if (globals_->labelHandler()->segmentationIdExists(class_id)) {
    label = globals_->labelHandler()->getLabelEntry(class_id);
  }

  if (label.label == PanopticLabel::kUnknown) {
    if (it->second.is_thing) {
      label.label = PanopticLabel::kInstance;
    } else {
      label.label = PanopticLabel::kBackground;
    }
  }

  // Allocate new submap.
  Submap* new_submap =
      submap_allocator_->allocateSubmap(submaps, input, input_id, label);
  new_submap->setClassName(label.name);
  new_submap->setName(label.name);
  return new_submap;
}

bool DetectronIDTracker::classesMatch(int input_id, int submap_class_id) {
  if (input_id == 0) {
    // The id 0 is used to denote no-predictions by detectron.
    return false;
  }
  auto it = labels_->find(input_id);
  if (it == labels_->end()) {
    // No known input label.
    return false;
  }

  int input_class_id = ClassNameManager::getGlobalInstance()->getClassID(
      it->second.category_name);
  return input_class_id == submap_class_id;
}

std::vector<float> DetectronIDTracker::getEmbeddingVector(int input_id) {
  if (input_id == 0) {
    // The id 0 is used to denote no-predictions by detectron.
    return std::vector<float>();
  }
  auto it = labels_->find(input_id);
  if (it == labels_->end()) {
    // No known input label.
    return std::vector<float>();
  }

  return it->second.embedding_vector;
}

float DetectronIDTracker::getEmbeddingScore(int input_id) {
  if (input_id == 0) {
    // The id 0 is used to denote no-predictions by detectron.
    return 0.0f;
  }
  auto it = labels_->find(input_id);
  if (it == labels_->end()) {
    // No known input label.
    return 0.0f;
  }

  return it->second.score;
}

}  // namespace panoptic_mapping
