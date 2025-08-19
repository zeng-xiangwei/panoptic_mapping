#include "panoptic_mapping/map_management/layer_manipulator.h"

#include <algorithm>

#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/merge_integration.h>

namespace panoptic_mapping {

void LayerManipulator::Config::checkParams() const {
  //  checkParamNE(error_threshold, 0.f, "error_threshold");
  checkParamGE(required_belonging_corners, 0, "required_belonging_corners");
  checkParamLE(required_belonging_corners, 8, "required_belonging_corners");
}

void LayerManipulator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("use_instance_classification", &use_instance_classification);
  setupParam("required_belonging_corners", &required_belonging_corners);
}

LayerManipulator::LayerManipulator(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  cube_index_offsets_ << 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0,
      0, 0, 1, 1, 1, 1;
}

void LayerManipulator::applyClassificationLayer(
    TsdfLayer* tsdf_layer, const ClassLayer& class_layer,
    float truncation_distance) const {
  // Check inputs.
  CHECK_NOTNULL(tsdf_layer);
  if (tsdf_layer->voxel_size() != class_layer.voxel_size() ||
      tsdf_layer->voxels_per_side() != class_layer.voxels_per_side()) {
    LOG(WARNING) << "Can not 'applyClassificationLayer' to layers that have "
                    "differing layouts.";
    return;
  }

  // Parse the tsdf layer.
  voxblox::BlockIndexList block_indices;
  tsdf_layer->getAllAllocatedBlocks(&block_indices);
  for (auto& block_index : block_indices) {
    TsdfBlock& tsdf_block = tsdf_layer->getBlockByIndex(block_index);
    const ClassBlock::ConstPtr class_block =
        class_layer.getBlockConstPtrByIndex(block_index);
    if (!class_block) {
      continue;
    }

    // Apply the voxel data.
    float min_distance = truncation_distance;
    bool was_updated = false;
    for (size_t i = 0; i < tsdf_block.num_voxels(); ++i) {
      voxblox::VoxelIndex voxel_index =
          tsdf_block.computeVoxelIndexFromLinearIndex(i);
      TsdfVoxel& tsdf_voxel = tsdf_block.getVoxelByLinearIndex(i);
      if (tsdf_voxel.weight <= 1.0e-6) {
        continue;
      }
      if (!class_block->getVoxelByLinearIndex(i).belongsToSubmap()) {
        // TODO(schmluk): Could get proper distance by looking up the surface.
        // TODO(schmluk): Could use probability to change the weights?
        tsdf_voxel.distance = truncation_distance;
        was_updated = true;
      } else {
        if (config_.required_belonging_corners > 0) {
          int belonging_corners = belongingCornersNum(tsdf_layer, class_layer,
                                                      block_index, voxel_index);
          if (belonging_corners <= config_.required_belonging_corners) {
            tsdf_voxel.distance = truncation_distance;
            was_updated = true;
          }
        }
        min_distance = std::min(tsdf_voxel.distance, min_distance);
      }
    }
    if (min_distance == truncation_distance) {
      // This block does not contain useful data anymore.
      tsdf_layer->removeBlock(block_index);
    } else if (was_updated) {
      tsdf_block.setUpdatedAll();
    }
  }
}

void LayerManipulator::mergeSubmapAintoB(const Submap& A, Submap* B) const {
  // TODO(schmluk): At the moment abuses the fact that all transforms are the
  //  identity and that equal classes have equal layer layout!

  if (A.hasClassLayer() != B->hasClassLayer()) {
    LOG(WARNING)
        << "Submap merging currently can only fuse submaps with class layers.";
    return;
  }
  const bool use_class_layer = A.hasClassLayer();

  // Currently just use the voxels...
  voxblox::BlockIndexList block_indices;
  A.getTsdfLayer().getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : block_indices) {
    TsdfBlock::Ptr tsdf_block_B =
        B->getTsdfLayerPtr()->allocateBlockPtrByIndex(block_index);
    tsdf_block_B->setUpdatedAll();
    const TsdfBlock::ConstPtr tsdf_block_A =
        A.getTsdfLayer().getBlockPtrByIndex(block_index);
    ClassBlock::ConstPtr class_block_A;
    ClassBlock::Ptr class_block_B;
    if (use_class_layer) {
      class_block_A = A.getClassLayer().getBlockConstPtrByIndex(block_index);
      class_block_B =
          B->getClassLayerPtr()->allocateBlockPtrByIndex(block_index);
    }

    for (size_t i = 0; i < tsdf_block_B->num_voxels(); ++i) {
      // Get the voxels and meta data.
      const TsdfVoxel& tsdf_voxel_A = tsdf_block_A->getVoxelByLinearIndex(i);
      TsdfVoxel& tsdf_voxel_B = tsdf_block_B->getVoxelByLinearIndex(i);
      const ClassVoxel* class_voxel_A = nullptr;
      ClassVoxel* class_voxel_B = nullptr;
      bool belongs_A = true;
      if (class_block_A) {
        class_voxel_A = &class_block_A->getVoxelByLinearIndex(i);
        belongs_A = class_voxel_A->belongsToSubmap();
      }
      bool belongs_B = true;
      if (class_block_B) {
        class_voxel_B = &class_block_B->getVoxelByLinearIndex(i);
        belongs_B = class_voxel_B->belongsToSubmap();
      }
      if (belongs_A && belongs_B) {
        voxblox::mergeVoxelAIntoVoxelB(tsdf_voxel_A, &tsdf_voxel_B);
        if (class_voxel_A && class_voxel_B) {
          // Voxels that belong to A and B are merged.
          class_voxel_B->mergeVoxel(*class_voxel_A);
        }
      } else if (belongs_A) {
        // If it only belongs to A but not to B overwrite B.
        tsdf_voxel_B.distance = tsdf_voxel_A.distance;
        tsdf_voxel_B.weight = tsdf_voxel_A.weight;
        tsdf_voxel_B.color = tsdf_voxel_A.color;
        if (class_voxel_A && class_voxel_B) {
          *class_voxel_B = *class_voxel_A;
        }
      }
      // If it does not belong to A or neither then no action is required.
    }
  }
}

void LayerManipulator::unprojectTsdfLayer(TsdfLayer* tsdf_layer) const {
  CHECK_NOTNULL(tsdf_layer);
  voxblox::EsdfIntegrator::Config config;
  voxblox::Layer<voxblox::EsdfVoxel> esdf_layer(tsdf_layer->voxel_size(),
                                                tsdf_layer->voxels_per_side());
  voxblox::EsdfIntegrator integrator(config, tsdf_layer, &esdf_layer);
  integrator.updateFromTsdfLayerBatch();
  voxblox::BlockIndexList block_indices;
  tsdf_layer->getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : block_indices) {
    TsdfBlock& tsdf_block = tsdf_layer->getBlockByIndex(block_index);
    const voxblox::Block<voxblox::EsdfVoxel>& esdf_block =
        esdf_layer.getBlockByIndex(block_index);
    for (size_t i = 0; i < tsdf_block.num_voxels(); ++i) {
      tsdf_block.getVoxelByLinearIndex(i).distance =
          esdf_block.getVoxelByLinearIndex(i).distance;
    }
  }
}

int LayerManipulator::belongingCornersNum(
    TsdfLayer* tsdf_layer, const ClassLayer& class_layer,
    const voxblox::BlockIndex& block_index,
    const voxblox::VoxelIndex& voxel_index) const {
  int belonging_corners_num = 0;
  TsdfBlock& tsdf_block = tsdf_layer->getBlockByIndex(block_index);
  const ClassBlock::ConstPtr class_block =
      class_layer.getBlockConstPtrByIndex(block_index);
  int voxels_per_side = tsdf_layer->voxels_per_side();
  for (unsigned int i = 0; i < 8; ++i) {
    voxblox::VoxelIndex corner_index = voxel_index + cube_index_offsets_.col(i);

    if (tsdf_block.isValidVoxelIndex(corner_index)) {
      const TsdfVoxel& voxel = tsdf_block.getVoxelByVoxelIndex(corner_index);
      if (voxel.weight < 1.0e-6) {
        continue;
      }
      if (class_block->getVoxelByVoxelIndex(corner_index).belongsToSubmap()) {
        belonging_corners_num++;
      }
    } else {
      // We have to access a different block.
      voxblox::BlockIndex block_offset = voxblox::BlockIndex::Zero();

      for (unsigned int j = 0u; j < 3u; j++) {
        if (corner_index(j) < 0) {
          block_offset(j) = -1;
          corner_index(j) = corner_index(j) + voxels_per_side;
        } else if (corner_index(j) >=
                   static_cast<voxblox::IndexElement>(voxels_per_side)) {
          block_offset(j) = 1;
          corner_index(j) = corner_index(j) - voxels_per_side;
        }
      }

      voxblox::BlockIndex neighbor_index =
          tsdf_block.block_index() + block_offset;

      if (tsdf_layer->hasBlock(neighbor_index)) {
        const TsdfBlock& neighbor_block =
            tsdf_layer->getBlockByIndex(neighbor_index);

        CHECK(neighbor_block.isValidVoxelIndex(corner_index));
        const TsdfVoxel& voxel =
            neighbor_block.getVoxelByVoxelIndex(corner_index);

        if (voxel.weight < 1.0e-6) {
          continue;
        }
        // The class blocks should always exist but just make sure.
        const ClassBlock::ConstPtr neighbor_class_block =
            class_layer.getBlockConstPtrByIndex(neighbor_index);
        if (!neighbor_class_block) {
          continue;
        }
        if (neighbor_class_block->getVoxelByVoxelIndex(corner_index)
                .belongsToSubmap()) {
          belonging_corners_num++;
        }
      }
    }
  }
  return belonging_corners_num;
}

}  // namespace panoptic_mapping
