#ifndef PANOPTIC_MAPPING_TOOLS_SINGLE_TSDF_FOR_UNDETECTED_H_
#define PANOPTIC_MAPPING_TOOLS_SINGLE_TSDF_FOR_UNDETECTED_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/globals.h"
#include "panoptic_mapping/integration/projective_tsdf_integrator.h"

namespace panoptic_mapping {

/**
 * @brief Integrator that integrates only undetected pixels into
 * a single submap for pure geometric TSDF reconstruction.
 */
class SingleTsdfForUndetected : public ProjectiveIntegrator {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Submap allocation config. This submap is used for pure geometric
    // reconstruction without classification.
    Submap::Config submap;

    // Standard integrator params.
    ProjectiveIntegrator::Config projective_integrator;

    // If true require a color image and update voxel colors.
    bool use_color = true;

    // 有效的相机深度范围，这个范围与常规语义建图流程中的范围不同，这个范围可以更大
    float min_range = 0.3f;
    float max_range = 5.0f;

    Config() { setConfigName("SingleTsdfForUndetected"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  SingleTsdfForUndetected(const Config& config,
                          std::shared_ptr<Globals> globals);
  ~SingleTsdfForUndetected() override = default;

  void processInput(SubmapCollection* submaps, InputData* input) override;

 protected:
  // Override methods specific to the single TSDF update for undetected
  // regions.
  void allocateNewBlocks(Submap* map, InputData* input);

  void updateBlock(Submap* submap, InterpolatorBase* interpolator,
                   const voxblox::BlockIndex& block_index,
                   const Transformation& T_C_S,
                   const InputData& input) const override;

  bool updateVoxel(InterpolatorBase* interpolator, TsdfVoxel* voxel,
                   const Point& p_C, const InputData& input,
                   const int submap_id, const bool is_free_space_submap,
                   const float truncation_distance, const float voxel_size,
                   ClassVoxel* class_voxel = nullptr,
                   ScoreVoxel* score_voxel = nullptr) const override;

 private:
  const Config config_;

 public:
  static config_utilities::Factory::RegistrationRos<
      TsdfIntegratorBase, SingleTsdfForUndetected, std::shared_ptr<Globals>>
      registration_;

 private:
  std::unique_ptr<Camera> camera_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_SINGLE_TSDF_FOR_UNDETECTED_H_
