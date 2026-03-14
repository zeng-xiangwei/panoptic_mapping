#ifndef PANOPTIC_MAPPING_MAP_CLASSIFICATION_EXPONENTIAL_MOVING_BINARY_COUNT_H_
#define PANOPTIC_MAPPING_MAP_CLASSIFICATION_EXPONENTIAL_MOVING_BINARY_COUNT_H_

#include <memory>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/map/classification/class_layer_impl.h"
#include "panoptic_mapping/map/classification/class_voxel.h"
#include "panoptic_mapping/map/classification/moving_binary_count.h"

namespace panoptic_mapping {

/**
 * @brief Binary classification by simple counting, where ID 0 indicates the
 * voxel belongs. Using exponential decay calculate belong count.
 */
struct ExponentialMovingBinaryCountVoxel : public MovingBinaryCountVoxel {
 public:
  // Implement interfaces.
  void incrementCount(const int id, const float weight = 1.f) override;

  // Data.
  float exp_lambda_ = 0.9f;
  float score_ = 0.5f;
};

class ExponentialMovingBinaryCountLayer
    : public ClassLayerImpl<ExponentialMovingBinaryCountVoxel> {
 public:
  struct Config : public config_utilities::Config<Config> {
    Config() { setConfigName("ExponentialMovingBinaryCountLayer"); }

   protected:
    void fromRosParam() override {}
    void printFields() const override {}
  };

  ExponentialMovingBinaryCountLayer(const Config& config,
                                    const float voxel_size,
                                    const int voxels_per_side);

  // Overwrite these method since we only need half a word per voxel.
  bool saveBlockToStream(BlockIndex block_index,
                         std::fstream* outfile_ptr) const override;
  bool addBlockFromProto(const voxblox::BlockProto& block_proto) override;

  ClassVoxelType getVoxelType() const override;
  std::unique_ptr<ClassLayer> clone() const override;
  static std::unique_ptr<ClassLayer> loadFromStream(
      const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
      uint64_t* /* tmp_byte_offset_ptr */);

 protected:
  const Config config_;

 public:
  static config_utilities::Factory::RegistrationRos<
      ClassLayer, ExponentialMovingBinaryCountLayer, float, int>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_CLASSIFICATION_MOVING_BINARY_COUNT_H_
