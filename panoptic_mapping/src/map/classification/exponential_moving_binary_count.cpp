#include "panoptic_mapping/map/classification/exponential_moving_binary_count.h"

#include <limits>
#include <memory>
#include <vector>

namespace panoptic_mapping {

void ExponentialMovingBinaryCountVoxel::incrementCount(const int id, const float weight) {
  // ID 0 is used for belonging voxels.
  if (id == 0u) {
    score_ = exp_lambda_ * score_ + (1.f - exp_lambda_) * 1.f;
  } else {
    score_ = exp_lambda_ * score_ + (1.f - exp_lambda_) * 0.f;
  }

  belongs_count = 100 * score_;
  foreign_count = 100 - belongs_count;
}

config_utilities::Factory::RegistrationRos<ClassLayer, ExponentialMovingBinaryCountLayer,
                                           float, int>
    ExponentialMovingBinaryCountLayer::registration_("exponential_moving_binary_count");
ExponentialMovingBinaryCountLayer::ExponentialMovingBinaryCountLayer(const Config& config,
                                                                     const float voxel_size,
                                                                     const int voxels_per_side)
    : config_(config.checkValid()),
      ClassLayerImpl(voxel_size, voxels_per_side) {}

bool ExponentialMovingBinaryCountLayer::saveBlockToStream(
    BlockIndex block_index, std::fstream* outfile_ptr) const {
  CHECK_NOTNULL(outfile_ptr);
  auto block = layer_.getBlockPtrByIndex(block_index);
  if (!block) {
    return false;
  }

  // Save data.
  voxblox::BlockProto proto;
  proto.set_has_data(block->has_data());
  proto.set_voxels_per_side(block->voxels_per_side());
  proto.set_voxel_size(block->voxel_size());
  proto.set_origin_x(block->origin().x());
  proto.set_origin_y(block->origin().y());
  proto.set_origin_z(block->origin().z());
  uint16_t tmp_data;
  bool first_packet_half = true;
  int test = 0;
  for (size_t i = 0; i < block->num_voxels(); ++i) {
    const uint16_t data = static_cast<uint16_t>(
        block->getVoxelByLinearIndex(i).serializeVoxelToInt()[0]);
    // Always combine two voxels into a word. The number of voxels is always a
    // multiple of two.
    if (first_packet_half) {
      tmp_data = data;
      first_packet_half = false;
    } else {
      proto.add_voxel_data(int32FromTwoInt16(tmp_data, data));
      first_packet_half = true;
    }
  }
  if (!voxblox::utils::writeProtoMsgToStream(proto, outfile_ptr)) {
    LOG(ERROR) << "Could not write class block proto message to stream.";
    return false;
  }
  return true;
}

bool ExponentialMovingBinaryCountLayer::addBlockFromProto(
    const voxblox::BlockProto& block_proto) {
  // Check compatibility.
  if (!isCompatible(block_proto, *this)) {
    return false;
  }

  // Add (potentially replace) the block.
  const Point origin(block_proto.origin_x(), block_proto.origin_y(),
                     block_proto.origin_z());
  layer_.removeBlockByCoordinates(origin);
  auto block = layer_.allocateNewBlockByCoordinates(origin);

  // Read the data, where two voxels are unpacked from each word.
  std::vector<uint32_t> data;
  data.resize(block_proto.voxel_data_size() * 2);
  size_t index = 0;
  for (uint32_t word : block_proto.voxel_data()) {
    const std::pair<uint16_t, uint16_t> datum = twoInt16FromInt32(word);
    data[index] = datum.first;
    data[index + 1] = datum.second;
    index += 2;
  }

  // Load the voxels.
  index = 0;
  for (size_t i = 0; i < block->num_voxels(); ++i) {
    if (!reinterpret_cast<ClassVoxel&>(block->getVoxelByLinearIndex(i))
             .deseriliazeVoxelFromInt(data, &index)) {
      LOG(WARNING) << "Could not serialize voxel from data.";
      return false;
    }
  }
  return true;
}

ClassVoxelType ExponentialMovingBinaryCountLayer::getVoxelType() const {
  return ClassVoxelType::kExponentialMovingBinaryCount;
}

std::unique_ptr<ClassLayer> ExponentialMovingBinaryCountLayer::clone() const {
  return std::make_unique<ExponentialMovingBinaryCountLayer>(*this);
}

std::unique_ptr<ClassLayer> ExponentialMovingBinaryCountLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  // Nothing special needed to configure for binary counts.
  return std::make_unique<ExponentialMovingBinaryCountLayer>(
      ExponentialMovingBinaryCountLayer::Config(), submap_proto.voxel_size(),
      submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
