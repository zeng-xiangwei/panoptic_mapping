/**
AUTHOR:       Lukas Schmid <schmluk@mavt.ethz.ch>
AFFILIATION:  Autonomous Systems Lab (ASL), ETH Zürich
SOURCE:       https://github.com/ethz-asl/config_utilities
VERSION:      1.2.2
LICENSE:      BSD-3-Clause

Copyright 2020 Autonomous Systems Lab (ASL), ETH Zürich.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Raises a redefined warning if different versions are used. v=MMmmPP.
#define CONFIG_UTILITIES_VERSION 010202

#ifndef CONFIG_UTILITIES_CORE_HPP_
#define CONFIG_UTILITIES_CORE_HPP_

#include <algorithm>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace config_utilities {

/**
 * ==================== Settings ====================
 */
namespace internal {
/**
 * @brief Global Settings for how config_utilities based configs behave. These
 * can be dynamically set and changed throughout a project.
 */
struct GlobalSettings {
  GlobalSettings(const GlobalSettings& other) = delete;
  GlobalSettings& operator=(const GlobalSettings& other) = delete;
  static GlobalSettings& instance() {
    static GlobalSettings settings;
    return settings;
  }

  // Printing Settings.
  // Width of the 'toString()' output of configs.
  unsigned int print_width = 80;

  // Indent after which values are printed.
  unsigned int print_indent = 30;

  // Indent for nested configs.
  unsigned int subconfig_indent = 3;

  // If true, indicate which values are identical to the default.
  bool indicate_default_values = true;

  // If true, also display the unit of each parameter where provided.
  bool indicate_units = true;

 private:
  GlobalSettings() = default;
};
}  // namespace internal

// Access.
inline internal::GlobalSettings& GlobalSettings() {
  return internal::GlobalSettings::instance();
}

/**
 * ==================== Utilities ====================
 */

/**
 * @brief Add required command line arguments if needed. Keeps memory over the
scope of existence for this class.
*/
class RequiredArguments {
 public:
  explicit RequiredArguments(int* argc, char*** argv,
                             const std::vector<std::string>& arguments) {
    // Read old arguments to string.
    std::vector<std::string> old_args;
    old_args.reserve(*argc);
    for (int i = 0; i < *argc; ++i) {
      old_args.emplace_back((*argv)[i]);
    }

    // Detect new arguments.
    std::vector<std::string> added_args;
    for (const std::string& arg : arguments) {
      if (std::find(old_args.begin(), old_args.end(), arg) == old_args.end()) {
        added_args.push_back(arg);
      }
    }

    // Write old arguments.
    *argc = old_args.size() + added_args.size();
    argv_aux_ = std::vector<std::unique_ptr<char>>(*argc);
    for (int i = 0; i < old_args.size(); ++i) {
      argv_aux_[i].reset(new char[std::strlen((*argv)[i]) + 1]);
      strcpy(argv_aux_[i].get(), (*argv)[i]);
    }

    // Write new arguments.
    for (int i = old_args.size(); i < *argc; ++i) {
      argv_aux_[i].reset(
          new char[std::strlen(added_args[i - old_args.size()].c_str()) +
                   1]);  // Extra char for null-terminated string.
      strcpy(argv_aux_[i].get(), added_args[i - old_args.size()].c_str());
    }

    // Write argv.
    argv_.reserve(*argc);
    for (int i = 0; i < *argc; ++i) {
      argv_[i] = argv_aux_[i].get();
    }
    *argv = argv_.data();
  }

 private:
  std::vector<char*> argv_;
  std::vector<std::unique_ptr<char>> argv_aux_;
};

/**
 * ==================== Internal Utilities ====================
 */
namespace internal {
// Printing utilities.
inline std::string printCenter(const std::string& text, int width,
                               char symbol) {
  int first = std::max((width - static_cast<int>(text.length()) - 2) / 2, 0);
  std::string result = std::string(first, symbol) + " " + text + " ";
  result += std::string(std::max(width - static_cast<int>(result.length()), 0),
                        symbol);
  return result;
}
/**
 * @brief Resolve namespaces to ROS standards.
 *
 * @param param_namespace The current param namespace of the config.
 * @param sub_namespace Sub-name space to read parameters from.
 * @param private_namespace Private namespace of the ROS node.
 * @return std::string Resolved global ROS namespace.
 */
inline std::string resolveNameSpaceROS(const std::string& param_namespace,
                                       const std::string& sub_namespace,
                                       const std::string& private_namespace) {
  // Update the sub-namespace. Default to same ns. Leading "/" for global ns.
  // Leading "~" for private ns.
  if (sub_namespace.empty()) {
    return param_namespace;
  } else if (sub_namespace.front() == '/') {
    return sub_namespace;
  } else if (sub_namespace.front() == '~') {
    return private_namespace + sub_namespace.substr(1);
  } else {
    return param_namespace + "/" + sub_namespace;
  }
}

// Type verification.
// NOTE(schmluk): This is not particularly pretty but works well for varying
// template types.
struct ConfigInternalVerificator {};

template <typename T>
inline bool isConfig(const T* candidate) {
  try {
    throw candidate;
  } catch (const ConfigInternalVerificator*) {
    return true;
  } catch (...) {
  }
  return false;
}

struct VariableConfigVerificator {};

template <typename T>
inline bool isVariableConfig(const T* candidate) {
  try {
    throw candidate;
  } catch (const VariableConfigVerificator*) {
    return true;
  } catch (...) {
  }
  return false;
}

// Setup param type. These contain all params of the nodehandle and the full
// namespaces, the params "_name_space" and "_name_space_private" are reserved
// for the target namespace to look up data. Namespacing adheres to ROS
// standards with "/" for global and "~" for private namespaces.
using ParamMap = std::unordered_map<std::string, YAML::Node>;
inline ParamMap deepClone(const ParamMap& yaml_node_map) {
  ParamMap result;
  for (const auto& [key, node] : yaml_node_map) {
    result[key] = YAML::Clone(node);
  }
  return result;
}

template <typename T>
inline bool yamlCast(const YAML::Node& node, T* param = nullptr) {
  if (!node.IsDefined()) {
    return false;
  }
  try {
    if (param) {
      *param = node.as<T>();
    }
    return true;
  } catch (...) {
    return false;
  }
}

// Forward declarations.
struct ConfigInternal;
inline std::unique_ptr<ConfigInternal> copyConfig(const ConfigInternal* config);

// Defines virtual interfaces implemented in the corresponding VariableConfig
// part.
struct VariableConfigInternal : public VariableConfigVerificator {
 public:
  // Constructors.
  VariableConfigInternal() = default;
  VariableConfigInternal(const VariableConfigInternal& other) {
    config_ = copyConfig(other.config_.get());
    type_ = other.type_;
  }
  VariableConfigInternal& operator=(const VariableConfigInternal& other) {
    config_ = copyConfig(other.config_.get());
    type_ = other.type_;
    return *this;
  }
  VariableConfigInternal(VariableConfigInternal&& other) {
    config_ = copyConfig(other.config_.get());
    type_ = other.type_;
  }
  VariableConfigInternal& operator=(VariableConfigInternal&& other) {
    if (&other != this) {
      config_ = copyConfig(other.config_.get());
      type_ = other.type_;
    }
    return *this;
  }

  // Access. These methods wrap the inspection tools for the contained config.
  virtual bool isValid() const { return false; }

  virtual std::string toString() const { return "Variable config not setup."; }

  const std::string& type() const { return type_; }

  bool isSetup() const { return config_ != nullptr; }

 protected:
  friend ConfigInternal;
  std::string type_ = "Not Setup";
  std::unique_ptr<ConfigInternal> config_ = nullptr;

  virtual void createConfig(const ParamMap& params, bool optional) {};
};
}  // namespace internal

/**
 * ==================== ConfigChecker ====================
 */

/**
 * @brief Utility tool to make checking configs easier and more readable.
 * Instantiate the checker, then execute all checks, and get a summary of all
 * checks eventually.
 */
class ConfigChecker {
 public:
  explicit ConfigChecker(std::string module_name)
      : name_(std::move(module_name)),
        print_width_(GlobalSettings().print_width) {}

  /**
   * @brief Return whether the config checker is valid, i.e. whether none of the
   * executed checks failed.
   *
   * @param print_warnings If true, print the warnings to console. Default:
   * false.
   */
  bool isValid(bool print_warnings = false) const {
    if (warnings_.empty()) {
      return true;
    }
    if (print_warnings) {
      std::string sev = "Warning: ";
      int length = print_width_ - sev.length();
      std::string warning =
          "\n" + internal::printCenter(name_, print_width_, '=');
      for (std::string w : warnings_) {
        std::string line = sev;
        while (w.length() > length) {
          line.append(w.substr(0, length));
          w = w.substr(length);
          warning.append("\n" + line);
          line = std::string(sev.length(), ' ');
        }
        warning.append("\n" + line + w);
      }
      warning = warning + "\n" + std::string(print_width_, '=');
      LOG(WARNING) << warning;
    }
    return false;
  }

  /**
   * @brief Enforce that the config is valid. This will terminate the program if
   * invalid.
   */
  void checkValid() const { CHECK(isValid(true)); }

  /**
   * @brief Undoes all previously executed checks.
   */
  void reset() { warnings_.clear(); }

  /**
   * @brief Execute a greater than (GT) check, i.e. param > value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkGT(const T& param, const T& value, const std::string& name) {
    if (param <= value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected > '" << value << "' (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a greater equal (GE) check, i.e. param >= value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkGE(const T& param, const T& value, const std::string& name) {
    if (param < value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected >= '" << value << "' (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a less than (LT) check, i.e. param < value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkLT(const T& param, const T& value, const std::string& name) {
    if (param >= value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected < '" << value << "' (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a less equal (LE) check, i.e. param <= value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkLE(const T& param, const T& value, const std::string& name) {
    if (param > value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected <= '" << value << "' (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute an equal (Eq) check, i.e. param == value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkEq(const T& param, const T& value, const std::string& name) {
    if (param != value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected to be '" << value << "' (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a not equal (NE) check, i.e. param != value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkNE(const T& param, const T& value, const std::string& name) {
    if (param == value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected to be different from '" << value
         << "'.";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a condition check, i.e. whether condition is true.
   *
   * @param condition condition to be checked.
   * @param warning Message to be reported in the error summary.
   */
  void checkCond(bool condition, const std::string& warning) {
    if (!condition) {
      warnings_.emplace_back(warning);
    }
  }

  /**
   * @brief Set the printing width of the error summary.
   *
   * @param width New width.
   */
  void setPrintWidth(int width) { print_width_ = width; }

 private:
  const std::string name_;
  std::vector<std::string> warnings_;
  int print_width_;
};

/**
 * ==================== ConfigInternal ====================
 */
namespace internal {
// Base class for internal use.
struct ConfigInternal : public ConfigInternalVerificator {
 public:
  // Constructors.
  explicit ConfigInternal(std::string name)
      : name_(std::move(name)), meta_data_(new MetaData()) {}

  ConfigInternal(const ConfigInternal& other)
      : name_(other.name_), meta_data_(new MetaData(*(other.meta_data_))) {}

  ConfigInternal& operator=(const ConfigInternal& other) {
    name_ = other.name_;
    meta_data_.reset(new MetaData(*(other.meta_data_)));
    return *this;
  }

  ConfigInternal(ConfigInternal&& other)
      : name_(other.name_), meta_data_(std::move(other.meta_data_)) {}

  ConfigInternal& operator=(ConfigInternal&& other) {
    if (&other != this) {
      name_ = other.name_;
      meta_data_ = std::move(other.meta_data_);
    }
    return *this;
  }

  // Public interaction with configs.

  /**
   * @brief Evaluates whether any of the validity conditions are violated.
   * Returns true if the config is valid.
   *
   * @param print_warnings True: print invalid parameters to console. Default:
   * false.
   */
  bool isValid(bool print_warnings = false) const {
    meta_data_->checker = std::make_unique<ConfigChecker>(name_);
    meta_data_->checker->setPrintWidth(GlobalSettings::instance().print_width);
    meta_data_->print_warnings = print_warnings;
    checkParams();
    bool result = meta_data_->checker->isValid(print_warnings);
    meta_data_->checker.reset(nullptr);
    return result;
  }

  /**
   * @brief Produces a printable summary of the config as string, containing its
   * name and all parameter values.
   */
  std::string toString() const {
    std::string result =
        internal::printCenter(name_, GlobalSettings::instance().print_width,
                              '=') +
        "\n" + toStringInternal(meta_data_->indent) + "\n" +
        std::string(GlobalSettings::instance().print_width, '=');
    meta_data_->messages.reset(nullptr);
    return result;
  };

 protected:
  // Implementable setup tool.
  /**
   * @brief This function is called when 'checkValid()' is called or after a
   * config is created from ROS or similar. Implement this to initialize default
   * parameters that depend on other values.
   */
  virtual void initializeDependentVariableDefaults() {}

  /**
   * @brief This function is called when validity is checked. Implement this
   * function by adding all checks using the protected checkParamXX functions.
   */
  virtual void checkParams() const {}

  /**
   * @brief This function is called when the config is printed to string.
   * Implement this function by adding all printField functions.
   */
  virtual void printFields() const {
    if (!meta_data_->merged_setup_already_used &&
        !meta_data_->use_printing_to_get_values) {
      meta_data_->messages->emplace_back(
          std::string(meta_data_->indent, ' ')
              .append("The 'printFields()' method is not implemented for '" +
                      name_ + "'."));
    }
  }

  /**
   * @brief This function is called when a config is created from ROS param or
   * similar. Implement this function by adding all rosParam functions.
   */
  virtual void fromRosParam() {
    if (!meta_data_->merged_setup_already_used) {
      LOG(WARNING) << "fromRosParam() is not implemented for '" << name_
                   << "', no parameters will be loaded.";
    }
  }

  /**
   * @brief This function combines printFields and fromRosParma to avoid code
   * duplication. Implement this function by ading all setupParam functions.
   */
  virtual void setupParamsAndPrinting() {
    // If this is overridden this won't be set and will precede fromRosParam
    // and printFields.
    meta_data_->merged_setup_already_used = false;
  }

  // General Tools.
  /**
   * @brief Set the name of the config to be displayed in checks and prints.
   * Default is the typeid-name of the object.
   */
  void setConfigName(const std::string& name) { name_ = name; }

  // Checking Tools.
  /**
   * @brief Execute a greater than (GT) check, i.e. param > value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkParamGT(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamGT()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkGT(param, value, name);
  }

  /**
   * @brief Execute a greater equal (GE) check, i.e. param >= value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkParamGE(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamGE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkGE(param, value, name);
  }

  /**
   * @brief Execute a less than (LT) check, i.e. param < value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkParamLT(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamLT()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkLT(param, value, name);
  }

  /**
   * @brief Execute a less equal (LE) check, i.e. param <= value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkParamLE(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamLE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkLE(param, value, name);
  }

  /**
   * @brief Execute a equal (Eq) check, i.e. param == value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkParamEq(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamEq()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkEq(param, value, name);
  }

  /**
   * @brief Execute a not equal (NE) check, i.e. param != value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkParamNE(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamNE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkNE(param, value, name);
  }

  /**
   * @brief Execute a condition check, i.e. whether condition is true.
   *
   * @param condition condition to be checked.
   * @param warning Message to be reported in the error summary.
   */
  void checkParamCond(bool condition, const std::string& warning) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamCond()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkCond(condition, warning);
  }

  /**
   * @brief Execute a config check, i.e. whether config is valid.
   *
   * @param config Config to be validated.
   */
  void checkParamConfig(const internal::ConfigInternal& config) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamConfig()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    if (!config.isValid(meta_data_->print_warnings)) {
      meta_data_->checker->checkCond(
          false, "Member config '" + config.name_ + "' is not valid.");
    }
  }

  /**
   * @brief Execute a config check, i.e. whether config is valid.
   *
   * @param config Config to be validated.
   */
  void checkParamConfig(const internal::VariableConfigInternal& config) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamConfig()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    if (!config.isSetup()) {
      meta_data_->checker->checkCond(false,
                                     "Variable member config is not setup.");
    } else {
      if (!config.config_->isValid(meta_data_->print_warnings)) {
        meta_data_->checker->checkCond(
            false, "Variable member config '" + config.config_->name_ +
                       "' (Variable Config: " + config.type() +
                       ") is not valid.");
      }
    }
  }

  // Printing Tools.
  /**
   * @brief Print this parameter in the config toString summary. The parameter
   * type T is required to implement the << operator.
   *
   * @tparam T Type of the parameter to be printed.
   * @param name Name of the parameter to be displayed.
   * @param field Value of the parameter to be retrieved.
   * @param unit Unit of the parameter. If not set nothing will be displayed.
   */
  template <typename T>
  void printField(const std::string& name, const T& field,
                  const std::string& unit = "") const {
    if (isConfig(&field)) {
      if (!meta_data_->use_printing_to_get_values) {
        // The default values only apply for non-config fields, as these will be
        // managed internally when printing the sub-config.
        printConfigInternal(name, (const internal::ConfigInternal*)&field);
      }
    } else if (isVariableConfig(&field)) {
      if (!meta_data_->use_printing_to_get_values) {
        // The default values only apply for non-config fields, as these will be
        // managed internally when printing the sub-config.
        auto var = (const internal::VariableConfigInternal*)&field;
        if (var->config_ != nullptr) {
          printConfigInternal(name + " (Variable Config: " + var->type_ + ")",
                              var->config_.get());

        } else {
          printFieldInternal(name, "Uninitialized Variable Config.");
        }
      }
    } else {
      std::stringstream ss;
      ss << field;
      printFieldInternal(name, ss.str(), unit);
    }
  }

  void printField(const std::string& name, const bool& field,
                  const std::string& unit = "") const {
    std::string val = "False";
    if (field) {
      val = "True";
    }
    printFieldInternal(name, val, unit);
  }

  template <typename T>
  void printField(const std::string& name, const std::vector<T>& field,
                  const std::string& unit = "") const {
    std::stringstream ss;
    ss << "[";
    size_t offset = 0;
    for (const T& value : field) {
      ss << value << ", ";
      offset = 2;
    }
    std::string s = ss.str();
    s = s.substr(0, s.length() - offset).append("]");
    printFieldInternal(name, s, unit);
  }

  template <typename T>
  void printField(const std::string& name,
                  const std::map<std::string, T>& field,
                  const std::string& unit = "") const {
    std::stringstream ss;
    ss << "{";
    size_t offset = 0;
    for (auto it = field.begin(); it != field.end(); ++it) {
      ss << it->first << ": " << it->second << ", ";
      offset = 2;
    }
    std::string s = ss.str();
    s = s.substr(0, s.length() - offset).append("}");
    printFieldInternal(name, s, unit);
  }

  /**
   * @brief Print this text un-indented in config toString summaries.
   *
   * @param text Text to be displayed.
   */
  void printText(const std::string& text) const {
    if (!meta_data_->messages) {
      if (!meta_data_->merged_setup_currently_active &&
          !meta_data_->use_printing_to_get_values) {
        LOG(WARNING) << "'printText()' calls are only allowed within the "
                        "'printFields()' method.";
      }
      return;
    }
    if (!meta_data_->use_printing_to_get_values) {
      meta_data_->messages->emplace_back(
          std::string(meta_data_->indent, ' ').append(text));
    }
  }

 private:
  friend std::unordered_map<std::string, std::string> getValues(
      const ConfigInternal& config);

  std::unordered_map<std::string, std::string> getValues() const {
    // This is only used within printing, so meta data exists.
    meta_data_->default_values =
        std::make_unique<std::unordered_map<std::string, std::string>>();
    meta_data_->use_printing_to_get_values = true;
    meta_data_->merged_setup_already_used = true;
    meta_data_->merged_setup_set_params = false;
    meta_data_->merged_setup_currently_active = true;
    // NOTE(schmluk): setupParamsAndPrinting() does not modify 'this' in
    // printing mode.
    ((ConfigInternal*)this)->setupParamsAndPrinting();
    printFields();

    std::unordered_map<std::string, std::string> result =
        *(meta_data_->default_values);
    meta_data_->default_values.reset(nullptr);
    meta_data_->use_printing_to_get_values = false;
    meta_data_->merged_setup_currently_active = false;
    return result;
  }

  void printFieldInternal(const std::string& name, const std::string& field,
                          const std::string& unit = "") const {
    // Use these calls to extract the string values of all fields.
    if (meta_data_->use_printing_to_get_values) {
      meta_data_->default_values->insert(std::make_pair(name, field));
      return;
    }

    // Check meta-data is valid.
    if (!meta_data_->messages) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'printField()' calls are only allowed within the "
                        "'printFields()' method.";
      }
      return;
    }

    std::string f = field;
    if (meta_data_->default_values) {
      auto it = meta_data_->default_values->find(name);
      if (it != meta_data_->default_values->end()) {
        if (it->second == field) {
          f.append(" (default)");
        }
      }
    }

    // The header is the field name.
    std::string header = std::string(meta_data_->indent, ' ') + name;
    if (GlobalSettings::instance().indicate_units && !unit.empty()) {
      header += " [" + unit + "]: ";
    } else {
      header += ": ";
    }
    while (header.length() > GlobalSettings::instance().print_width) {
      // Linebreaks for too long lines.
      meta_data_->messages->emplace_back(
          header.substr(0, GlobalSettings::instance().print_width));
      header = header.substr(GlobalSettings::instance().print_width);
    }
    if (header.length() < GlobalSettings::instance().print_indent) {
      header.append(std::string(
          GlobalSettings::instance().print_indent - header.length(), ' '));
    } else if (GlobalSettings::instance().print_width - header.length() <
               f.length()) {
      meta_data_->messages->emplace_back(header);
      header = std::string(GlobalSettings::instance().print_indent, ' ');
    }

    // First line could be shorter.
    int length = GlobalSettings::instance().print_width - header.length();
    if (f.length() > length) {
      meta_data_->messages->emplace_back(header + f.substr(0, length));
      f = f.substr(length);

      // Fill the rest.
      length = GlobalSettings::instance().print_width -
               GlobalSettings::instance().print_indent;
      while (f.length() > length) {
        meta_data_->messages->emplace_back(
            std::string(GlobalSettings::instance().print_indent, ' ') +
            f.substr(0, length));
        f = f.substr(length);
      }
      meta_data_->messages->emplace_back(
          std::string(GlobalSettings::instance().print_indent, ' ') +
          f.substr(0, length));
    } else {
      meta_data_->messages->emplace_back(header.append(f));
    }
  }

  void printConfigInternal(const std::string& name,
                           const internal::ConfigInternal* field) const {
    meta_data_->messages->emplace_back(std::string(meta_data_->indent, ' ') +
                                       name + ":");
    CHECK_NOTNULL(field);
    meta_data_->messages->emplace_back(field->toStringInternal(
        meta_data_->indent + GlobalSettings::instance().subconfig_indent));
  }

  std::string toStringInternal(int indent) const {
    const int indent_prev = meta_data_->indent;
    meta_data_->indent = indent;

    meta_data_->messages = std::make_unique<std::vector<std::string>>();
    meta_data_->merged_setup_set_params = false;
    meta_data_->merged_setup_currently_active = true;

    // Create the default values if required.
    if (GlobalSettings::instance().indicate_default_values) {
      auto defaults = getDefaultConfig();
      defaults->initializeDependentVariableDefaults();
      meta_data_->default_values =
          std::make_unique<std::unordered_map<std::string, std::string>>(
              defaults->getValues());
      meta_data_->use_printing_to_get_values = true;
      meta_data_->merged_setup_already_used = true;

      // NOTE: setupParamsAndPrinting() does not modify 'this' in printing mode.
      ((ConfigInternal*)this)->setupParamsAndPrinting();
      printFields();
    }

    meta_data_->use_printing_to_get_values = false;
    meta_data_->merged_setup_already_used = true;

    // NOTE: setupParamsAndPrinting() does not modify 'this' in printing mode.
    ((ConfigInternal*)this)->setupParamsAndPrinting();
    printFields();
    std::string result;
    for (const std::string& msg : *(meta_data_->messages)) {
      result.append("\n" + msg);
    }
    if (!result.empty()) {
      result = result.substr(1);
    }
    meta_data_->messages.reset(nullptr);
    meta_data_->default_values.reset(nullptr);
    meta_data_->indent = indent_prev;
    meta_data_->merged_setup_currently_active = false;

    return result;
  };

  void setupFromParamMap(const internal::ParamMap& params) {
    param_namespace_ = "/";
    if (params.find("_name_space") != params.end()) {
      yamlCast(params.at("_name_space"), &param_namespace_);
    }

    // Read params
    meta_data_->params = &params;
    meta_data_->merged_setup_already_used = true;
    meta_data_->merged_setup_set_params = true;
    meta_data_->merged_setup_currently_active = true;
    setupParamsAndPrinting();
    fromRosParam();
    meta_data_->params = nullptr;
    initializeDependentVariableDefaults();
  }

  template <typename T>
  void rosParamInternal(const std::string& name, T* param) {
    CHECK_NOTNULL(param);
    // Check scope and param map are valid.
    if (!meta_data_->params) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                        "'fromRosParam()' method, no param will be loaded.";
      }
      return;
    }
    // Check the param is set.
    auto it = meta_data_->params->find(param_namespace_ + "/" + name);
    if (it == meta_data_->params->end()) {
      return;
    }
    // Set the param.
    if (!internal::yamlCast(it->second, param)) {
      LOG(WARNING) << name_ << ": param '" << name
                   << "' is set but could not be read.";
    }
  }

  template <typename T>
  void rosParamMapInternal(const std::string& name,
                           std::map<std::string, T>* param) {
    CHECK_NOTNULL(param);
    // Check scope and param map are valid.
    if (!meta_data_->params) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                        "'fromRosParam()' method, no param will be loaded.";
      }
      return;
    }
    // Check the param is set.
    std::map<std::string, T> values;
    const std::string prefix = param_namespace_ + "/" + name + "/";
    for (const auto& v : *(meta_data_->params)) {
      if (v.first.find(prefix) != 0) {
        continue;
      }
      std::string key = v.first.substr(prefix.length());

      if (key.find('/') == std::string::npos) {
        T value;
        if (!internal::yamlCast(v.second, &value)) {
          LOG(WARNING) << name_ << ": param '" << name
                       << "' is set but could not be read.";
          return;
        }
        values[key] = value;
      }
    }
    *param = values;
  }

  template <typename T>
  void setupParamInternal(const std::string& name, T* param,
                          const std::string& unit) {
    if (meta_data_->merged_setup_set_params) {
      rosParam(name, param);
    } else {
      printField(name, *param, unit);
    }
  }

 protected:
  // These are explicitly overloaded for agreement with ROS-params and to
  // prevent messy compilation errors.
  /**
   * @brief Retrieve a parameter from ROS or similar.
   *
   * @param name Name of the parameter in the ROS-parameter server.
   * @param param Variable to store the retrieved value in.
   */
  void rosParam(const std::string& name, int* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, float* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, double* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, bool* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::string* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::vector<int>* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::vector<double>* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::vector<float>* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::vector<bool>* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::vector<std::string>* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::map<std::string, int>* param) {
    this->rosParamMapInternal(name, param);
  }

  void rosParam(const std::string& name, std::map<std::string, double>* param) {
    this->rosParamMapInternal(name, param);
  }

  void rosParam(const std::string& name, std::map<std::string, float>* param) {
    this->rosParamMapInternal(name, param);
  }

  void rosParam(const std::string& name, std::map<std::string, bool>* param) {
    this->rosParamMapInternal(name, param);
  }

  void rosParam(const std::string& name,
                std::map<std::string, std::string>* param) {
    this->rosParamMapInternal(name, param);
  }

  void rosParam(ConfigInternal* config, const std::string& sub_namespace = "") {
    CHECK_NOTNULL(config);
    // Check scope and param map are valid.
    if (!meta_data_->params) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                        "'fromRosParam()' method, no param will be loaded.";
      }
      return;
    }

    // Update the sub-namespace. Default to same ns. Leading "/" for global ns.
    // Leading "~" for private ns.
    internal::ParamMap params = internal::deepClone(*(meta_data_->params));

    params["_name_space"] =
        resolveNameSpaceROS(param_namespace_, sub_namespace,
                            params["_name_space_private"].as<std::string>());
    setupConfigFromParamMap(params, config);
  }

  void rosParam(VariableConfigInternal* config,
                const std::string& sub_namespace = "", bool optional = true) {
    CHECK_NOTNULL(config);
    // Resolve the param namespace and get param map.
    internal::ParamMap params = internal::deepClone(*(meta_data_->params));

    params["_name_space"] =
        resolveNameSpaceROS(param_namespace_, sub_namespace,
                            params["_name_space_private"].as<std::string>());

    // Create the desired config.
    config->createConfig(params, optional);
  }

  /**
   * @brief Extract the name space that was used when retrieving parameter
   * values.
   */
  std::string rosParamNameSpace() {
    // Check scope and param map are valid.
    if (!meta_data_->params) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING)
            << "'rosParamNameSpace()' calls are only allowed within the "
               "'fromRosParam()' method, no param will be loaded.";
      }
    }
    return param_namespace_;
  }

  /**
   * @brief Set the name and storage variable for a parameter. These will be
   * used for both the fromRosParam parameter retrieval and printField parameter
   * printing.
   *
   * @param name Name of the parameter to be retrieved and displayed.
   * @param param Value to be stored and read.
   * @param unit Unit of the parameter displayed when printing. By default
   * nothing will be displayed.
   */
  void setupParam(const std::string& name, int* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, float* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, double* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, bool* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, std::string* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, std::vector<int>* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, std::vector<double>* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, std::vector<float>* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, std::vector<bool>* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, std::vector<std::string>* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, std::map<std::string, int>* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, std::map<std::string, double>* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, std::map<std::string, float>* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name, std::map<std::string, bool>* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  void setupParam(const std::string& name,
                  std::map<std::string, std::string>* param,
                  const std::string& unit = "") {
    this->setupParamInternal(name, param, unit);
  }

  /**
   * @brief Set the name and storage variable for a parameter that is itself a
   * config_utilities config. These will be used for both the fromRosParam
   * parameter retrieval and printField parameter printing.
   *
   * @param name Name of the parameter to be retrieved and displayed.
   * @param config Value to be stored and read.
   * @param sub_namespace If set, all values for the child config will be
   * retrieved from the given sub-namespace.
   */
  void setupParam(const std::string& name, ConfigInternal* config,
                  const std::string& sub_namespace = "") {
    if (meta_data_->merged_setup_set_params) {
      rosParam(config, sub_namespace);
    } else {
      printField(name, *config);
    }
  }

  /**
   * @brief Set the name and storage variable for a parameter that is a
   * config_utilities VariableConfig. These will be used for both the
   * fromRosParam parameter retrieval and printField parameter printing.
   *
   * @param name Name of the parameter to be retrieved and displayed.
   * @param config Value to be stored and read.
   * @param sub_namespace If set, all values for the variable config will be
   * retrieved from the given sub-namespace.
   */
  void setupParam(const std::string& name, VariableConfigInternal* config,
                  const std::string& sub_namespace = "", bool optional = true) {
    if (meta_data_->merged_setup_set_params) {
      rosParam(config, sub_namespace, optional);
    } else {
      printField(name, *config);
    }
  }

 private:
  friend void setupConfigFromParamMap(const internal::ParamMap& params,
                                      ConfigInternal* config);
  friend std::unique_ptr<ConfigInternal> copyConfig(
      const ConfigInternal* config);

  virtual std::unique_ptr<internal::ConfigInternal> getDefaultConfig()
      const = 0;
  virtual std::unique_ptr<internal::ConfigInternal> copy() const = 0;

  struct MetaData {
    // temporary tools
    std::unique_ptr<ConfigChecker> checker;
    std::unique_ptr<std::vector<std::string>> messages;
    std::unique_ptr<std::unordered_map<std::string, std::string>>
        default_values;
    const internal::ParamMap* params = nullptr;

    // settings and variables
    int indent = 0;  // Only used for nested printing.
    bool print_warnings = false;
    bool merged_setup_already_used = false;
    bool merged_setup_set_params = false;
    bool merged_setup_currently_active = false;
    bool use_printing_to_get_values = false;

    MetaData() = default;
    MetaData(const MetaData& other) { indent = other.indent; }
  };

  std::string name_;
  std::string param_namespace_ = "~";
  std::unique_ptr<MetaData> meta_data_;
};  // namespace internal

// This is a dummy operator, configs provide toString().
inline std::ostream& operator<<(std::ostream& os, const ConfigInternal&) {
  return os;
}

inline std::ostream& operator<<(std::ostream& os,
                                const VariableConfigVerificator&) {
  return os;
}

/**
 * ==================== Exposure Utilities ===================
 */

inline void setupConfigFromParamMap(const ParamMap& params,
                                    ConfigInternal* config) {
  CHECK_NOTNULL(config);
  config->setupFromParamMap(params);
}

inline std::unique_ptr<ConfigInternal> copyConfig(
    const ConfigInternal* config) {
  if (config == nullptr) {
    return nullptr;
  } else {
    return config->copy();
  }
}
}  // namespace internal

/**
 * ==================== Config ====================
 */
/**
 * @brief Base class for configs to inherit from. To define a custom config, it
 * needs to template itself,
 * i.e. class MyConfig : public config_utilities::Config<MyConfig>
 */
template <typename ConfigT>
struct Config : public internal::ConfigInternal {
 public:
  // Construction.
  Config() : ConfigInternal(typeid(ConfigT).name()) {}

  /**
   * @brief Enforce that a config is valid. This will terminate the program if
   * constraints are violated. Runs initializeDependentVariableDefaults() before
   * checking.
   *
   * @return ConfigT Copy of the validated config.
   */
  ConfigT checkValid() const {
    // Returns a copy of the config in the const case.
    ConfigT result(*static_cast<const ConfigT*>(this));
    return result.checkValid();
  }

  /**
   * @brief Enforce that a config is valid. This will terminate the program if
   * constraints are violated. Runs initializeDependentVariableDefaults() before
   * checking.
   *
   * @return ConfigT The validated config.
   */
  ConfigT& checkValid() {
    // Returns a mutable reference.
    initializeDependentVariableDefaults();
    CHECK(isValid(true));
    return *static_cast<ConfigT*>(this);
  }

 private:
  std::unique_ptr<internal::ConfigInternal> getDefaultConfig() const override {
    return std::make_unique<ConfigT>();
  }

  std::unique_ptr<internal::ConfigInternal> copy() const override {
    return std::make_unique<ConfigT>(*static_cast<const ConfigT*>(this));
  }
};

/**
 * ==================== Factory ====================
 */
class Factory {
 public:
  /**
   * @brief Allocate these structs statically do add entries to the factory for
   * run-time creation.
   *
   * @tparam BaseT Type of the base class to be registered. The base classes are
   * queried for creation.
   * @tparam DerivedT Type of the derived class to register.
   * @tparam Args Other constructor arguments. Notice that each unique set of
   * constructor arguments will result in a different base-entry in the factory.
   * @param type String identifier to look up and to create this derived type.
   */
  template <class BaseT, class DerivedT, typename... Args>
  struct Registration {
    explicit Registration(const std::string& type) {
      ModuleMap<BaseT, Args...>::instance().template addEntry<DerivedT>(type);
    }
  };

  /**
   * @brief Allocate these structs statically do add entries to the ROS-factory
   * for run-time creation.
   *
   * @tparam BaseT Type of the base class to be registered. The base classes are
   * queried for creation.
   * @tparam DerivedT Type of the derived class to register.
   * @tparam Args Other constructor arguments. The first constructor argument
   * for all derived classes is required to be a <typename DerivedT::Config> and
   * not listed in the arguments. Notice that each unique set of constructor
   * arguments will result in a different base-entry in the factory.
   * @param type String identifier to look up and to create this derived type.
   */
  template <class BaseT, class DerivedT, typename... Args>
  struct RegistrationRos {
    explicit RegistrationRos(const std::string& type) {
      std::cout << "Registering " << type << std::endl;
      ModuleMap<BaseT, Args...>::instance().template addEntryRos<DerivedT>(
          type);
    }
  };

  /**
   * @brief Query the factory to create a dervied type.
   *
   * @tparam BaseT Type of the base class to query for.
   * @tparam Args Other constructor arguments. Notice that each unique set of
   * constructor arguments will result in a different base-entry in the factory.
   * @param type String identifier of the derived type to create.
   * @param args Other constructor arguments.
   * @return std::unique_ptr<BaseT> Unique pointer of type base that contains
   * the derived object.
   */
  template <class BaseT, typename... Args>
  static std::unique_ptr<BaseT> create(const std::string& type, Args... args) {
    ModuleMap<BaseT, Args...>& module = ModuleMap<BaseT, Args...>::instance();
    std::stringstream ss;
    ((ss << typeid(args).name() << ", "), ...);
    std::string type_info = ss.str();
    if (!type_info.empty()) {
      type_info = " and constructor arguments '" +
                  type_info.substr(0, type_info.size() - 2) + "'";
    } else {
      type_info = "";
    }
    if (module.map.empty()) {
      LOG(ERROR) << "Cannot create a module of type '" << type
                 << "': No modules registered to the factory for base '"
                 << typeid(BaseT).name() << "'" << type_info
                 << ". Register modules using a static Registration struct.";
      return nullptr;
    }
    auto it = module.map.find(type);
    if (it == module.map.end()) {
      std::string module_list;
      for (const auto& entry : module.map) {
        module_list.append(entry.first + ", ");
      }
      module_list = module_list.substr(0, module_list.size() - 2);
      LOG(ERROR) << "No module of type '" << type
                 << "' registered to the factory for base '"
                 << typeid(BaseT).name() << "'" << type_info
                 << ". Registered are: " << module_list << ".";
      return nullptr;
    }
    return std::unique_ptr<BaseT>(it->second(args...));
  }

  /**
   * @brief Query the factory to create a dervied type from a param map.
   *
   * @tparam BaseT Type of the base class to query for.
   * @tparam Args Other constructor arguments. Notice that each unique set of
   * constructor arguments will result in a different base-entry in the factory.
   * @param params Param map to create the derived type from. Needs to contain a
   * param named 'type' in the active namespace.
   * @param args Other constructor arguments.
   * @return std::unique_ptr<BaseT> Unique pointer of type base that contains
   * the derived object.
   */
  template <class BaseT, typename... Args>
  static std::unique_ptr<BaseT> create(const internal::ParamMap& params,
                                       Args... args) {
    ModuleMap<BaseT, Args...>& module = ModuleMap<BaseT, Args...>::instance();
    std::stringstream ss;
    ((ss << typeid(args).name() << ", "), ...);
    std::string type_info = ss.str();
    if (!type_info.empty()) {
      type_info = " and constructor arguments '" +
                  type_info.substr(0, type_info.size() - 2) + "'";
    } else {
      type_info = "";
    }

    // Check the namespace.
    auto it = params.find("_name_space");
    if (it == params.end()) {
      LOG(ERROR)
          << "ROS factory creation requires the ParamMap to have a namespace.";
      return nullptr;
    }
    const std::string ns = it->second.as<std::string>();

    // Get the type from param.
    it = params.find(ns + "/type");
    if (it == params.end()) {
      LOG(ERROR) << "ROS factory creation requires the param 'type' to be set "
                    "in namespace '"
                 << ns << "'.";
      return nullptr;
    }
    const std::string type = it->second.as<std::string>();

    // Check the source module exists.
    if (module.map_ros.empty()) {
      LOG(ERROR) << "Cannot create a module of type '" << type
                 << "': No modules registered to the factory for base '"
                 << typeid(BaseT).name() << "'" << type_info
                 << ". Register modules using a static Registration struct.";
      return nullptr;
    }
    auto it2 = module.map_ros.find(type);
    if (it2 == module.map_ros.end()) {
      std::string module_list;
      for (const auto& entry : module.map_ros) {
        module_list.append(entry.first + ", ");
      }
      module_list = module_list.substr(0, module_list.size() - 2);
      LOG(ERROR) << "No module of type '" << type
                 << "' registered to the factory for base '"
                 << typeid(BaseT).name() << "'" << type_info
                 << ". Registered are: " << module_list << ".";
      return nullptr;
    }

    // Get the config and create the target.
    return std::unique_ptr<BaseT>(it2->second(params, args...));
  }

  template <class BaseT>
  static std::unique_ptr<internal::ConfigInternal> createConfig(
      const internal::ParamMap& params, bool optional) {
    auto module = Factory::ModuleMapConfig<BaseT>::instance();

    // Check the namespace.
    auto it = params.find("_name_space");
    if (it == params.end()) {
      LOG(ERROR)
          << "ROS factory creation requires the ParamMap to have a namespace.";
      return nullptr;
    }
    const std::string ns = it->second.as<std::string>();

    // Get the type from param.
    it = params.find(ns + "/type");
    if (it == params.end()) {
      if (!optional) {
        LOG(ERROR)
            << "ROS factory creation requires the param 'type' to be set "
               "in namespace '"
            << ns << "'.";
      }
      return nullptr;
    }
    const std::string type = it->second.as<std::string>();

    // Check the source module exists.
    if (module.map_config.empty()) {
      LOG(ERROR) << "Cannot create a module of type '" << type
                 << "': No modules registered to the factory for base '"
                 << typeid(BaseT).name()
                 << "'. Register modules using a static Registration struct.";
      return nullptr;
    }
    auto it2 = module.map_config.find(type);
    if (it2 == module.map_config.end()) {
      std::string module_list;
      for (const auto& entry : module.map_config) {
        module_list.append(entry.first + ", ");
      }
      module_list = module_list.substr(0, module_list.size() - 2);
      LOG(ERROR) << "No module of type '" << type
                 << "' registered to the factory for base '"
                 << typeid(BaseT).name() << "'. Registered are: " << module_list
                 << ".";
      return nullptr;
    }

    // Get the config.
    return it2->second(params);
  }

 protected:
  // Separate module map that does only needs the base type.
  template <class BaseT>
  struct ModuleMapConfig {
   public:
    using FactoryMethod =
        std::function<std::unique_ptr<internal::ConfigInternal>(
            const internal::ParamMap& params)>;
    // Singleton access.
    static ModuleMapConfig& instance() {
      static ModuleMapConfig instance_;
      return instance_;
    }

    // The map.
    std::unordered_map<std::string, FactoryMethod> map_config;

   private:
    ModuleMapConfig() = default;
  };

  // Module map for factory creation.
  template <class BaseT, typename... Args>
  struct ModuleMap {
   public:
    using FactoryMethod = std::function<BaseT*(Args... args)>;
    using FactoryMethodRos =
        std::function<BaseT*(const internal::ParamMap& params, Args... args)>;
    // Singleton access.
    static ModuleMap& instance() {
      static ModuleMap instance_;
      return instance_;
    }

    // Add entries.
    template <class DerivedT>
    void addEntry(const std::string& type) {
      if (map.find(type) != map.end()) {
        LOG(ERROR) << "Cannot register already existent type '" << type
                   << "' for <DerivedT>='" << typeid(DerivedT).name()
                   << "' to factory for base '" << typeid(BaseT).name() << "'.";
      } else {
        map.insert(std::make_pair(
            type, [](Args... args) { return new DerivedT(args...); }));
      }
    }

    template <class DerivedT>
    void addEntryRos(const std::string& type) {
      if (map_ros.find(type) != map_ros.end()) {
        LOG(ERROR) << "Cannot register already existent type '" << type
                   << "' for <DerivedT>='" << typeid(DerivedT).name()
                   << "' to factory for base '" << typeid(BaseT).name() << "'.";
      } else {
        map_ros.insert(std::make_pair(
            type,
            [type](const internal::ParamMap& params, Args... args) -> BaseT* {
              typename DerivedT::Config config;
              auto config_ptr =
                  dynamic_cast<Config<typename DerivedT::Config>*>(&config);
              if (!config_ptr) {
                LOG(ERROR)
                    << "Cannot create '" << type << "' with <DerivedT>='"
                    << typeid(DerivedT).name()
                    << "': 'DerivedT::Config' needs to exist and inherit from "
                       "'config_utilities::Config<DerivedT::Config>'.";
                return nullptr;
              }
              internal::setupConfigFromParamMap(params, config_ptr);
              return new DerivedT(config, args...);
            }));
        Factory::ModuleMapConfig<BaseT>& modules_config =
            Factory::ModuleMapConfig<BaseT>::instance();
        modules_config.map_config.insert(std::make_pair(
            type,
            [type](const internal::ParamMap& params)
                -> std::unique_ptr<internal::ConfigInternal> {
              auto config = std::make_unique<typename DerivedT::Config>();
              auto config_ptr =
                  dynamic_cast<Config<typename DerivedT::Config>*>(
                      config.get());
              if (!config_ptr) {
                LOG(ERROR)
                    << "Cannot create config for '" << type
                    << "' with <DerivedT>='" << typeid(DerivedT).name()
                    << "': 'DerivedT::Config' needs to exist and inherit from "
                       "'config_utilities::Config<DerivedT::Config>'.";
                return nullptr;
              }
              internal::setupConfigFromParamMap(params, config_ptr);
              return config;
            }));
      }
    }
    // The maps.
    std::unordered_map<std::string, FactoryMethod> map;
    std::unordered_map<std::string, FactoryMethodRos> map_ros;

   private:
    ModuleMap() = default;
  };
};

/**
 * ==================== Variable Config ====================
 */

/**
 * @brief Wrapper that can contain a variable config to create a BaseT module
 * further downstream. The contained config needs to be of type
 * config_utilities::Config<DerivedT::Config>.
 *
 * @tparam BaseT Base class of the derived object for which a config is
 * contained.
 */
template <typename BaseT>
class VariableConfig : public internal::VariableConfigInternal {
 public:
  bool isValid() const override {
    if (config_ != nullptr) {
      return config_->isValid();
    }
    return false;
  }

  std::string toString() const override {
    if (config_ != nullptr) {
      return config_->toString();
    } else {
      return "Variable config not setup.";
    }
  }
  /**
   * @brief Creation tool that acts like the RosFactory to create a DerivedT
   * based on the stored config.
   *
   * @tparam Args Additional contructor arguments required for the BaseT.
   */
  template <typename... Args>
  std::unique_ptr<BaseT> create(Args... args) const {
    return Factory::create<BaseT>(params_, args...);
  }

 private:
  void createConfig(const internal::ParamMap& params, bool optional) override {
    config_ = Factory::createConfig<BaseT>(params, optional);
    if (config_) {
      std::string type_key =
          params.at("_name_space").as<std::string>() + "/type";
      type_ = params.at(type_key).as<std::string>();
      params_ = internal::deepClone(params);
    }
  };

  // TODO(schmluk): This is an ugly hack but needs more refactoring of the
  // factory system and naming to make this more clean and gerenal.
  internal::ParamMap params_;
};

}  // namespace config_utilities
#endif  // CONFIG_UTILITIES_CORE_HPP_

#ifndef CONFIG_UTILITIES_YAML_HPP_
#define CONFIG_UTILITIES_YAML_HPP_
/**
 * ==================== YAML Tools ====================
 */
namespace config_utilities {
namespace internal {

inline void convertYamlNodeToParamMap(const YAML::Node& node,
                                      const std::string& path,
                                      ParamMap& params) {
  if (node.IsMap()) {
    for (const auto& kv : node) {
      std::string key = kv.first.as<std::string>();
      std::string new_path = path.empty() ? "/" + key : path + "/" + key;
      convertYamlNodeToParamMap(kv.second, new_path, params);
    }
  } else {
    if (params.count(path) == 0) {
      params[path] = YAML::Clone(node);
    } else {
      LOG(ERROR) << "YAML parameter '" << path << "' already exists!";
    }
  }
}

inline std::pair<YAML::Node, bool> findNode(const YAML::Node& cur_node,
                                            const std::string& path) {
  if (path.empty() || path == "/") {
    return {cur_node, true};
  }

  std::string remaining = (path[0] == '/') ? path.substr(1) : path;
  size_t pos = remaining.find('/');
  std::string first_token =
      (pos == std::string::npos) ? remaining : remaining.substr(0, pos);

  bool has_key = false;
  if (cur_node && cur_node.IsMap()) {
    for (const auto& kv : cur_node) {
      if (kv.first && kv.first.as<std::string>() == first_token) {
        has_key = true;
        break;
      }
    }
  }
  if (!has_key) {
    return {cur_node, false};
  }

  if (pos != std::string::npos) {
    std::string next_remaining = remaining.substr(pos + 1);
    return findNode(cur_node[first_token], next_remaining);
  } else {
    return {cur_node[first_token], true};
  }
}

inline ParamMap getParamMapFromYaml(const YAML::Node& root,
                                    const std::string& filter_path = "") {
  ParamMap params;
  auto found_node = findNode(root, filter_path);
  bool found = found_node.second;
  YAML::Node cur_node = found_node.first;

  if (found && cur_node.IsDefined() && cur_node.IsMap()) {
    convertYamlNodeToParamMap(cur_node, filter_path, params);
  } else {
    LOG(WARNING) << "Specified YAML path does not exist: " << filter_path;
  }

  params["_name_space"] = filter_path;
  params["_name_space_private"] = filter_path;
  return params;
}

}  // namespace internal

/**
 * @brief Create a config from a given yaml node and key path
 *
 * @tparam ConfigT The config to create. ConfigT needs to inherit from
 * config_utilities::Config<ConfigT>.
 * @param root The root node of the yaml file
 * @param filter_path The key path to filter the yaml node.
 * @return ConfigT The created config.
 */
template <typename ConfigT>
ConfigT getConfigFromYaml(const YAML::Node& root,
                          const std::string& filter_path = "") {
  ConfigT config;
  if (!internal::isConfig(&config)) {
    LOG(ERROR) << "Cannot 'getConfigFromYaml()' for <ConfigT>='"
               << typeid(ConfigT).name()
               << "' that does not inherit from "
                  "'config_utilities::Config<ConfigT>'.";
    return config;
  }
  auto config_ptr = dynamic_cast<Config<ConfigT>*>(&config);
  if (!config_ptr) {
    LOG(ERROR) << "Cannot 'getConfigFromYaml()' for <ConfigT>='"
               << typeid(ConfigT).name()
               << "' that does not inherit from "
                  "'config_utilities::Config<ConfigT>'.";
    return config;
  }

  // Setup.
  internal::ParamMap params = internal::getParamMapFromYaml(root, filter_path);
  internal::setupConfigFromParamMap(params, config_ptr);
  return config;
}

template <typename T>
void addKeyValueToYaml(YAML::Node& root, const std::string& path,
                       const std::string& key, const T& value) {
  auto found_node = internal::findNode(root, path);
  bool found = found_node.second;
  YAML::Node cur_node = found_node.first;

  if (found) {
    cur_node[key] = value;
  } else {
    LOG(ERROR) << "Target node is not a map or invalid.";
  }
}

inline bool hasKeyInYamlPath(const YAML::Node& root, const std::string& path,
                             const std::string& key_to_check) {
  auto found_node = internal::findNode(root, path);
  bool found = found_node.second;
  YAML::Node cur_node = found_node.first;

  if (found && cur_node.IsMap()) {
    for (const auto& kv : cur_node) {
      if (kv.first && kv.first.as<std::string>() == key_to_check) {
        return true;
      }
    }
  }

  return false;
}

/**
 * ==================== YAML Factory ====================
 */

class FactoryYaml : protected Factory {
 public:
  /**
   * @brief Query the Yaml-factory to create a dervied type with its config
   * from a yaml node.
   */
  template <class BaseT, typename... Args>
  static std::unique_ptr<BaseT> create(const YAML::Node& root,
                                       const std::string& filter_path = "",
                                       Args... args) {
    // Get the config and create the target.
    internal::ParamMap params =
        internal::getParamMapFromYaml(root, filter_path);
    return Factory::create<BaseT>(params, args...);
  }
};

}  // namespace config_utilities

#endif  // CONFIG_UTILITIES_YAML_HPP_
