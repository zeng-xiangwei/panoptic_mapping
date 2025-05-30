#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_CHANGE_DETECTOR_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_CHANGE_DETECTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/common/globals.h"
#include "panoptic_mapping/common/input_data.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

class ChangeDetector {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Allowed disappear distance in meters where a point is still considered
    // visible in input data. Negative values are multiples of the voxel_size.
    float strong_disappear_threshold = -10;
    float weak_disappear_threshold = -2;

    // Minimum number of points required for a submap to considered disappear.
    int match_strong_disappear_points = 50;
    int match_weak_disappear_points = 50;

    // Minimum percentage of points required for a submap to considered
    // disappear.
    float match_strong_disappear_percentage = 0.6;
    float match_weak_disappear_percentage = 0.8;

    // For weak disappear point, the average distance from disappear points to
    // the projected pixel depth must larger than this value. Unit m, Negative
    // values are multiples of the voxel_size. This must larger than
    // weak_disappear_threshold
    float match_weak_average_distance = -3;

    // Number of threads used to perform change detection. Change detection is
    // submap-parallel.
    int detection_threads = std::thread::hardware_concurrency();
    Config() { setConfigName("ChangeDetector"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  ChangeDetector(const Config& config, std::shared_ptr<Globals> globals);
  virtual ~ChangeDetector() = default;

  // Check submap that in current view frustum but not visible
  // on input image, this submap is absent
  void checkSubmapCollectionVisibleByInputData(SubmapCollection* submaps,
                                               InputData* input);

 private:
  std::string checkSubmapVisibleByInputData(Submap* submap, InputData* input);

  const Config config_;
  const std::shared_ptr<Globals> globals_;
};

}  // namespace panoptic_mapping

#endif