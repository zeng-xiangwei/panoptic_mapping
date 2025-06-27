#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_NULL_MAP_MANAGER_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_NULL_MAP_MANAGER_H_

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/globals.h"
#include "panoptic_mapping/map/submap_collection.h"
#include "panoptic_mapping/map_management/map_manager_base.h"

namespace panoptic_mapping {

/**
 * @brief The null map manager is a default module that does not execute any
 * actions.
 */
class NullMapManager : public MapManagerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    Config() { setConfigName("NullMapManager"); }

   protected:
    void setupParamsAndPrinting() override;
  };

  NullMapManager(const Config& config, std::shared_ptr<Globals> globals)
      : config_(config), globals_(std::move(globals)) {}
  virtual ~NullMapManager() = default;

  // The null map manager does not execute any actions.
  void tick(SubmapCollection* submaps, InputData* input) override {}
  void finishMapping(SubmapCollection* submaps) override {}

 private:
  const Config config_;
  const std::shared_ptr<Globals> globals_;

 public:
  static config_utilities::Factory::RegistrationRos<
      MapManagerBase, NullMapManager, std::shared_ptr<Globals>>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_NULL_MAP_MANAGER_H_
