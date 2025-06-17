#ifndef PANOPTIC_MAPPING_LABELS_MINI_CSV_LABEL_HANDLER_H_
#define PANOPTIC_MAPPING_LABELS_MINI_CSV_LABEL_HANDLER_H_

#include <string>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/labels/label_entry.h"
#include "panoptic_mapping/labels/label_handler_base.h"

namespace panoptic_mapping {

/**
 * @brief This label handler reads a csv file to get the labels.
 * For auto generate class IDs
 */
class MiniCsvLabelHandler : public LabelHandlerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // File name to read the labels from.
    std::string file_name;
    Config() { setConfigName("MiniCsvLabelHandler"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit MiniCsvLabelHandler(const Config& config, bool print_config = true);
  ~MiniCsvLabelHandler() override = default;

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<LabelHandlerBase,
                                                    MiniCsvLabelHandler>
      registration_;
  void readLabelsFromFile();
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_LABELS_CSV_LABEL_HANDLER_H_
