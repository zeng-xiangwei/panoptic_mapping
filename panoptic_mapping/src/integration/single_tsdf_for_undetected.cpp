#include "panoptic_mapping/integration/single_tsdf_for_undetected.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {
config_utilities::Factory::RegistrationRos<
    TsdfIntegratorBase, SingleTsdfForUndetected, std::shared_ptr<Globals>>
    SingleTsdfForUndetected::registration_("single_tsdf_for_undetected");

void SingleTsdfForUndetected::Config::checkParams() const {
  checkParamConfig(projective_integrator);
}

void SingleTsdfForUndetected::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("projective_integrator", &projective_integrator);
  setupParam("submap", &submap);
  setupParam("use_color", &use_color);
  setupParam("min_range", &min_range);
  setupParam("max_range", &max_range);
  setupParam("single_tsdf_submap_id", &single_tsdf_submap_id);
  setupParam("reconstruct_detected_objects", &reconstruct_detected_objects);
}

SingleTsdfForUndetected::SingleTsdfForUndetected(
    const Config& config, std::shared_ptr<Globals> globals)
    : config_(config.checkValid()),
      ProjectiveIntegrator(config.projective_integrator, std::move(globals),
                           false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  Camera::Config camera_config = globals_->camera()->getConfig();
  camera_config.max_range = config_.max_range;
  camera_config.min_range = config_.min_range;
  camera_ = std::make_unique<Camera>(camera_config);

  cam_config_ = &camera_->getConfig();

  // Setup all needed inputs.
  setRequiredInputs({InputData::InputType::kDepthImage,
                     InputData::InputType::kVertexMap,
                     InputData::InputType::kValidityImage,
                     InputData::InputType::kSegmentationImage});
  if (config_.use_color) {
    addRequiredInputs({InputData::InputType::kColorImage});
  }
}

void SingleTsdfForUndetected::processInput(SubmapCollection* submaps,
                                           InputData* input) {
  // 传入的 submaps 必须与语义建图的 submaps 区分开，是一个单独的 submap
  // collection，里面只应该有 1 个 submap
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK_NOTNULL(camera_);
  CHECK(inputIsValid(*input));

  Submap* single_tsdf_submap = nullptr;
  if (submaps->size() == 0) {
    Submap::Config submap_config;
    submap_config = config_.submap;
    if (submap_config.truncation_distance < 0.f) {
      submap_config.truncation_distance *= -submap_config.voxel_size;
    }
    single_tsdf_submap =
        submaps->createSubmap(submap_config, config_.single_tsdf_submap_id);
    submaps->setActiveFreeSpaceSubmapID(single_tsdf_submap->getID());
    LOG(INFO) << "Created new submap " << single_tsdf_submap->getID()
              << " for single tsdf integration.";
  }
  single_tsdf_submap =
      submaps->getSubmapPtr(submaps->getActiveFreeSpaceSubmapID());

  // Allocate all blocks in the map.
  auto t1 = std::chrono::high_resolution_clock::now();
  allocateNewBlocks(single_tsdf_submap, input);
  auto t2 = std::chrono::high_resolution_clock::now();

  // Find all active blocks that are in the field of view.
  voxblox::BlockIndexList block_lists = camera_->findVisibleBlocks(
      *single_tsdf_submap, input->T_M_C(), max_range_in_image_);
  std::vector<voxblox::BlockIndex> indices;
  indices.resize(block_lists.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    indices[i] = block_lists[i];
  }
  IndexGetter<voxblox::BlockIndex> index_getter(indices);
  const Transformation T_C_S =
      input->T_M_C().inverse() * single_tsdf_submap->getT_M_S();

  // Integrate in parallel.
  std::vector<std::future<void>> threads;
  for (int i = 0; i < config_.projective_integrator.integration_threads; ++i) {
    threads.emplace_back(std::async(
        std::launch::async,
        [this, &index_getter, single_tsdf_submap, input, i, T_C_S]() {
          voxblox::BlockIndex index;
          while (index_getter.getNextIndex(&index)) {
            this->updateBlock(single_tsdf_submap, interpolators_[i].get(),
                              index, T_C_S, *input);
          }
        }));
  }

  // Join all threads.
  for (auto& thread : threads) {
    thread.get();
  }
  auto t3 = std::chrono::high_resolution_clock::now();

  LOG_IF(INFO, config_.verbosity >= 3)
      << "Allocate: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
      << "ms, Integrate: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
      << "ms. For single tsdf submap " << submaps->getActiveFreeSpaceSubmapID()
      << ".";
}

void SingleTsdfForUndetected::updateBlock(
    Submap* submap, InterpolatorBase* interpolator,
    const voxblox::BlockIndex& block_index, const Transformation& T_C_S,
    const InputData& input) const {
  // Set up preliminaries.
  if (!submap->getTsdfLayer().hasBlock(block_index)) {
    LOG_IF(WARNING, config_.verbosity >= 1)
        << "Tried to access inexistent block '" << block_index.transpose()
        << "' in submap " << submap->getID() << ".";
    return;
  }
  TsdfBlock& block = submap->getTsdfLayerPtr()->getBlockByIndex(block_index);
  bool was_updated = false;
  const float voxel_size = block.voxel_size();
  const float truncation_distance = submap->getConfig().truncation_distance;
  const int submap_id = submap->getID();

  // Update all voxels.
  for (size_t i = 0; i < block.num_voxels(); ++i) {
    TsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
    const Point p_C = T_C_S * block.computeCoordinatesFromLinearIndex(
                                  i);  // Voxel center in camera frame.
    if (updateVoxel(interpolator, &voxel, p_C, input, submap_id, true,
                    truncation_distance, voxel_size, nullptr, nullptr)) {
      was_updated = true;
    }
  }

  if (was_updated) {
    block.setUpdatedAll();
  }
}

bool SingleTsdfForUndetected::updateVoxel(
    InterpolatorBase* interpolator, TsdfVoxel* voxel, const Point& p_C,
    const InputData& input, const int submap_id,
    const bool is_free_space_submap, const float truncation_distance,
    const float voxel_size, ClassVoxel* class_voxel,
    ScoreVoxel* score_voxel) const {
  // Compute the signed distance. This also sets up the interpolator.
  float sdf;
  if (!computeSignedDistance(p_C, interpolator, &sdf)) {
    return false;
  }
  if (sdf < -truncation_distance) {
    return false;
  }

  // Get the interpolated ID value at this pixel location.
  const int pixel_id = interpolator->interpolateID(input.idImageCopy());

  // Compute the weight of the measurement.
  const float weight = computeWeight(p_C, voxel_size, truncation_distance, sdf);

  // Truncate the sdf to the truncation band.
  sdf = std::min(truncation_distance, sdf);

  // Only merge color near the surface.
  if (std::abs(sdf) < truncation_distance) {
    const Color color = interpolator->interpolateColor(input.colorImage());
    updateVoxelValues(voxel, sdf, weight, &color);
    // Only integrate pixels that are undetected (id == 0).
    if (!config_.reconstruct_detected_objects && pixel_id != 0) {
      // Reset weight to avoid integrating detected areas.
      voxel->weight = 0.f;
    }
  } else {
    updateVoxelValues(voxel, sdf, weight);
  }

  return true;
}

void SingleTsdfForUndetected::allocateNewBlocks(Submap* map, InputData* input) {
  range_image_.setZero();
  max_range_in_image_ = 0.f;

  const Transformation T_S_C = map->getT_S_M() * input->T_M_C();
  const cv::Mat& id_image = input->idImageCopy();
  for (int v = 0; v < input->depthImage().rows; v++) {
    for (int u = 0; u < input->depthImage().cols; u++) {
      const int pixel_id = id_image.at<int32_t>(v, u);
      const cv::Vec3f& vertex = input->vertexMap().at<cv::Vec3f>(v, u);
      const Point p_C(vertex[0], vertex[1], vertex[2]);
      const float ray_distance = p_C.norm();
      range_image_(v, u) = ray_distance;
      if (ray_distance > cam_config_->max_range ||
          ray_distance < cam_config_->min_range) {
        continue;
      }
      max_range_in_image_ = std::max(max_range_in_image_, ray_distance);
      const Point p_S = map->getT_S_M() * input->T_M_C() * p_C;
      const voxblox::BlockIndex block_index =
          map->getTsdfLayer().computeBlockIndexFromCoordinates(p_S);
      const auto block =
          map->getTsdfLayerPtr()->allocateBlockPtrByIndex(block_index);
    }
  }
  max_range_in_image_ = std::min(max_range_in_image_, cam_config_->max_range);

  // Update the bounding volume.
  map->updateBoundingVolume();
}

}  // namespace panoptic_mapping
