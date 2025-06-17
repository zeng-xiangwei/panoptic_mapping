#include "panoptic_mapping/labels/mini_csv_label_handler.h"

#include <sys/stat.h>

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "panoptic_mapping/3rd_party/csv.h"
#include "panoptic_mapping/map/class_name_manager.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<LabelHandlerBase,
                                           MiniCsvLabelHandler>
    MiniCsvLabelHandler::registration_("mini_csv");

void MiniCsvLabelHandler::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("file_name", &file_name);
}

void MiniCsvLabelHandler::Config::checkParams() const {
  // Check the specified file exists.
  struct stat buffer;
  checkParamCond(stat(file_name.c_str(), &buffer) == 0,
                 "Target file '" + file_name + "' does not exist.");
}

MiniCsvLabelHandler::MiniCsvLabelHandler(const Config& config,
                                         bool print_config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
  // Setup the labels from csv file.
  readLabelsFromFile();
}

void MiniCsvLabelHandler::readLabelsFromFile() {
  io::CSVReader<2> in(config_.file_name);
  in.read_header(io::ignore_extra_column, "ClassName", "Size");

  bool read_row = true;
  std::vector<float> field_count(2, 0.f);
  int missed_count = -1;  // The header is also counter.
  while (read_row) {
    std::string name, size;
    read_row = in.read_row(name, size);

    // Write all found values to the label.
    LabelEntry label;
    if (!name.empty()) {
      label.name = name;
      field_count[0] += 1.f;
      label.class_id = ClassNameManager::getGlobalInstance()->getClassID(name);
    } else {
      missed_count += 1;
      continue;
    }
    if (!size.empty()) {
      label.size = size;
      field_count[1] += 1.f;
    }
    labels_[label.class_id] = std::make_unique<LabelEntry>(label);
  }

  // Cehck all labels valid.
  if (missed_count) {
    LOG(ERROR)
        << missed_count
        << " labels could not be read, the 'InstanceID' field needs to be set!";
  }

  // Required fields.
  const size_t num_labels = labels_.size();
  const std::vector<std::string> field_names = {"ClassName", "Size"};
  for (size_t i = 1; i < 2; ++i) {
    if (field_count[i] < num_labels) {
      LOG(WARNING) << "Required field '" << field_names[i]
                   << "' is not set for " << num_labels - field_count[i]
                   << " entries.";
    }
  }

  // Logging.
  std::stringstream info;
  info << "Read " << num_labels << "/" << num_labels + missed_count
       << " labels from '" << config_.file_name << ".";
  if (config_.verbosity >= 3) {
    info << " Read fields: ";
    for (size_t i = 0; i < 2; ++i) {
      info << "\n  -" << field_names[i]
           << std::string(12 - field_names[i].length(), ' ') << "("
           << std::fixed << std::setprecision(1)
           << field_count[i] / static_cast<float>(num_labels) * 100.f << "%)";
    }
  }
  LOG_IF(INFO, config_.verbosity >= 1) << info.str();
}

}  // namespace panoptic_mapping
