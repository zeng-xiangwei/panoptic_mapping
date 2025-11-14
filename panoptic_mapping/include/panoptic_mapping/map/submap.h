#ifndef PANOPTIC_MAPPING_MAP_SUBMAP_H_
#define PANOPTIC_MAPPING_MAP_SUBMAP_H_

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <voxblox/core/layer.h>
#include <voxblox/mesh/mesh_layer.h>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/integration/mesh_integrator.h"
#include "panoptic_mapping/map/classification/class_block.h"
#include "panoptic_mapping/map/classification/class_layer.h"
#include "panoptic_mapping/map/classification/class_voxel.h"
#include "panoptic_mapping/map/instance_id.h"
#include "panoptic_mapping/map/scores/score_block.h"
#include "panoptic_mapping/map/scores/score_layer.h"
#include "panoptic_mapping/map/scores/score_voxel.h"
#include "panoptic_mapping/map/submap_bounding_volume.h"
#include "panoptic_mapping/map/submap_id.h"
#include "panoptic_mapping/map/vllm_description.h"

namespace panoptic_mapping {

class LayerManipulator;

class Submap {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    // Size of one voxel in meters.
    float voxel_size = 0.1;

    // Size of the truncation band for TSDF in meters. Negative values indicate
    // multiples of the voxel size.
    float truncation_distance = -2;

    // Numbers of voxels per side of a voxel block. Needs to be a power of 2.
    int voxels_per_side = 16;

    // Config of the classification voxels to be used. Leave the config
    // uninitialized (not setting the 'type' param) can be used to not use any
    // classification.
    config_utilities::VariableConfig<ClassLayer> classification;

    // Config of the score voxels to be used. Leave the config
    // uninitialized (not setting the 'type' param) can be used to not use any
    // score integration.
    config_utilities::VariableConfig<ScoreLayer> scores;

    // Config of the mesh integrator.
    MeshIntegrator::Config mesh;

    // 使用固定的 frame_id，避免使用
    // tf，因为目前的适用的submap都是表示在同一个世界系下，且均为单位阵
    std::string frame_id = "world";

    // 特征向量的最大权重、最小权重，最小权重仅用来判断是否特征向量是否可用
    float max_embedding_weight = 1000.f;
    float min_embedding_weight = 1e-5f;

    Config() { setConfigName("Submap"); }

    // Utility tool that checks whether a classification layer was specified.
    bool useClassLayer() const;
    // Utility tool that checks whether a score layer was specified.
    bool useScoreLayer() const;

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
    void initializeDependentVariableDefaults() override;
  };

  // Construction.
  explicit Submap(
      const Config& config,
      SubmapIDManager* submap_id_manager = SubmapIDManager::getGlobalInstance(),
      InstanceIDManager* instance_id_manager =
          InstanceIDManager::getGlobalInstance());

  // This constructor is intended to allow deep copies of the submap collection,
  // moving the id to the new id managers.
  Submap(const Config& config, SubmapIDManager* submap_id_manager,
         InstanceIDManager* instance_id_manager, int submap_id);
  virtual ~Submap() = default;

  // Const accessors.
  const Config& getConfig() const { return config_; }
  int getID() const { return id_; }
  int getInstanceID() const { return instance_id_; }
  int getClassID() const { return class_id_; }
  const std::string& getClassName() const { return class_name_; }
  const std::vector<float>& getEmbeddingVector() const {
    return embedding_vector_;
  }
  float getEmbeddingWeight() const { return embedding_weight_; }
  float getEmbeddingScore() const { return embedding_score_; }
  PanopticLabel getLabel() const { return label_; }
  const std::string& getName() const { return name_; }
  const std::string& getFrameName() const { return frame_name_; }
  const TsdfLayer& getTsdfLayer() const { return *tsdf_layer_; }
  const ClassLayer& getClassLayer() const { return *class_layer_; }
  const ScoreLayer& getScoreLayer() const { return *score_layer_; }
  const voxblox::MeshLayer& getMeshLayer() const { return *mesh_layer_; }
  const Transformation& getT_M_S() const { return T_M_S_; }
  const Transformation& getT_S_M() const { return T_M_S_inv_; }
  bool isActive() const { return is_active_; }
  bool wasTracked() const { return was_tracked_; }
  bool hasClassLayer() const { return has_class_layer_; }
  bool hasScoreLayer() const { return has_score_layer_; }
  bool matchRedetection() const { return match_redetection_; }
  const std::vector<IsoSurfacePoint>& getIsoSurfacePoints() const {
    return iso_surface_points_;
  }
  ChangeState getChangeState() const { return change_state_; }
  const SubmapBoundingVolume& getBoundingVolume() const {
    return bounding_volume_;
  }

  // Modifying accessors.
  std::shared_ptr<TsdfLayer>& getTsdfLayerPtr() { return tsdf_layer_; }
  std::shared_ptr<ClassLayer>& getClassLayerPtr() { return class_layer_; }
  std::shared_ptr<ScoreLayer>& getScoreLayerPtr() { return score_layer_; }
  std::shared_ptr<voxblox::MeshLayer>& getMeshLayerPtr() { return mesh_layer_; }
  std::vector<IsoSurfacePoint>* getIsoSurfacePointsPtr() {
    return &iso_surface_points_;
  }
  SubmapBoundingVolume* getBoundingVolumePtr() { return &bounding_volume_; }
  int getDisappearCount() const { return disappear_count_; }
  VllmDescription getDescriptsByVllm() const { return descripts_by_vllm_; };
  bool getHasNewVllmDescripts() const { return has_new_vllm_descripts_; }
  std::vector<VllmRelationship>* getVllmRelationshipsPtr() {
    return &relationships_by_vllm_;
  }

  // Setters.
  void setDisappearCount(int count) { disappear_count_ = count; }
  void setT_M_S(const Transformation& T_M_S);
  void setInstanceID(int id) { instance_id_ = id; }
  void setLabel(PanopticLabel label) { label_ = label; }
  void setName(const std::string& name) { name_ = name; }
  void setFrameName(const std::string& name) { frame_name_ = name; }
  void setChangeState(ChangeState state) { change_state_ = state; }
  void setIsActive(bool is_active) { is_active_ = is_active; }
  void setWasTracked(bool was_tracked) { was_tracked_ = was_tracked; }
  void setMatchRedetection(bool match) { match_redetection_ = match; }
  void setDescriptsByVllm(const VllmDescription& descripts) {
    descripts_by_vllm_ = descripts;
  }

  void setHasNewVllmDescripts(bool has_new_vllm_descripts) {
    has_new_vllm_descripts_ = has_new_vllm_descripts;
  }

  /**
   * @brief Set the Class Name, auto generate unique class id
   *
   * @param class_name
   */
  void setClassName(const std::string& class_name);
  void setEmbeddingVector(const std::vector<float>& embedding_vector,
                          float score = 0.0);

  /**
   * @brief Update embedding vector. By average.
   *
   */
  void updateEmbeddingVector(const std::vector<float>& embedding_vector,
                             float score = 0.0);

  void addDisappearCount(int add = 1);

  // Processing.
  /**
   * @brief Set the submap status to inactive and update its status accordingly.
   */
  void finishActivePeriod();

  /**
   * @brief Update all dynamically computable quantities.
   *
   * @param only_updated_blocks If false, recompute all quantities from scratch.
   * If true, recompute based on what is flagged updated.
   */
  void updateEverything(bool only_updated_blocks = true);

  /**
   * @brief Update the bounding volume based on all allocated blocks.
   */
  void updateBoundingVolume();

  /**
   * @brief Update the mesh based on the current tsdf blocks. Set
   * only_updated_blocks true for incremental mesh updates, false for a full
   * re-computation.
   *
   * @param only_updated_blocks If false, recompute the mesh from scratch. If
   * true, update based on the updated(kMesh) flag of the TSDF layer.
   * @param use_class_layer Set to true to use the class layer if it is
   * available.
   */
  void updateMesh(bool only_updated_blocks = true, bool use_class_layer = true);

  /**
   * @brief Compute the iso-surface points of the submap based on its current
   * mesh. Currently all surface points are computed from scratch every time,
   * but since they are currently only computed when a submap is finished it
   * should be fine. This function utilizes the stored mesh so make sure
   * updateMesh is called earlier.
   */
  void computeIsoSurfacePoints();

  /**
   * @brief Removes non-belonging points from the TSDF and deletes the class
   * layer. Uses the provided manipulator to perform the class layer
   * integration.
   *
   * @param manipulator Manipulator used to carry out the application
   * of the class layer.
   * @param clear_class_layer True: erase the class layer. False: keep the class
   * layer for lookups, but no further manipulations.
   * @return True if any blocks remain, false if the TSDF map was cleared.
   */
  bool applyClassLayer(const LayerManipulator& manipulator,
                       bool clear_class_layer = true);

  /**
   * @brief Create a deep copy of the submap. Notice that new submapID and
   * instanceID managers need to be provided to not corrupt the ID counts. ID
   * counts will not be double checked so use with care.
   *
   * @param submap_id_manager Pointer to the new submapID manager that holds
   * this submap.
   * @param instance_id_manager Pointer to the new instanceID manager that holds
   * this submap.
   * @return Unique pointer holding the copy.
   */
  std::unique_ptr<Submap> clone(SubmapIDManager* submap_id_manager,
                                InstanceIDManager* instance_id_manager) const;

 private:
  friend class SubmapCollection;
  const Config config_;

  // Setup.
  void initialize();

  // IO.
  /**
   * @brief Serialize the submap to protobuf.
   *
   * @param proto The output protobuf object.
   */
  void getProto(SubmapProto* proto) const;

  /**
   * @brief Save the submap to file.
   *
   * @param outfile_ptr The file to write the protobuf data to.
   * @return Success of the saving operation.
   */
  bool saveToStream(std::fstream* outfile_ptr) const;

  /**
   * @brief Load the submap from file.
   *
   * @param proto_file_ptr File from where to read the protobuf data.
   * @param tmp_byte_offset_ptr Byte offset result, used to keep track where we
   * are in the file if necessary. NOTE(schmluk): Mostly unused, initialize to
   * 0.
   * @param id_manager Submap ID manager of the collection to laod the submap
   * into.
   * @param instance_manager Instance ID manager of the collection to laod the
   * submap into.
   * @return Unique pointer to the loaded submap.
   */
  static std::unique_ptr<Submap> loadFromStream(
      std::istream* proto_file_ptr, uint64_t* tmp_byte_offset_ptr,
      SubmapIDManager* id_manager = SubmapIDManager::getGlobalInstance(),
      InstanceIDManager* instance_manager =
          InstanceIDManager::getGlobalInstance());

  // Labels.
  const SubmapID id_;       // UUID
  InstanceID instance_id_;  // Per default sets up a new unique ID.
  int class_id_ = -1;
  std::string class_name_;
  PanopticLabel label_ = PanopticLabel::kUnknown;
  std::string name_ = "Unknown";
  std::vector<float> embedding_vector_;
  float embedding_score_ = 0.f;
  float embedding_weight_ = 0.f;  // 废弃该字段

  // State.
  bool is_active_ = true;
  bool was_tracked_ = true;  // Set to true by the id tracker if matched.
  bool has_class_layer_ = false;
  bool has_score_layer_ = false;
  bool match_redetection_ = false;
  ChangeState change_state_ = ChangeState::kNew;

  // Transformations.
  std::string frame_name_;
  Transformation T_M_S_;  // Transformation mission to submap.
  Transformation T_M_S_inv_;

  // Map.
  std::shared_ptr<TsdfLayer> tsdf_layer_;
  std::shared_ptr<ClassLayer> class_layer_;
  std::shared_ptr<ScoreLayer> score_layer_;
  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;
  std::vector<IsoSurfacePoint> iso_surface_points_;
  SubmapBoundingVolume bounding_volume_;

  // Processing.
  std::unique_ptr<MeshIntegrator> mesh_integrator_;

  // Change detection.
  int disappear_count_ = 0;

  // VLLM提供的物体属性描述
  VllmDescription descripts_by_vllm_;
  // 物体与物体之间的位置关系，这里的 from_id 就是该 submap id，to_id
  // 是另一个submap id
  std::vector<VllmRelationship> relationships_by_vllm_;
  // TODO: 暂时设计为描述与关系只要有一个有变化，这个字段就设为true
  bool has_new_vllm_descripts_ = false;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_SUBMAP_H_
