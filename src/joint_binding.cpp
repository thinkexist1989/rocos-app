// Copyright 2026, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#include "joint_binding.hpp"

#include <algorithm>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "logger.hpp"

namespace rocos {

namespace {

std::string JoinStrings(const std::vector<std::string>& values) {
  std::ostringstream oss;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) oss << ", ";
    oss << values[i];
  }
  return oss.str();
}

std::string JoinDriveIds(const std::vector<int32_t>& values) {
  std::ostringstream oss;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) oss << ", ";
    oss << values[i];
  }
  return oss.str();
}

}  // namespace

Result JointBinding::Configure(const std::vector<std::string>& model_joint_names,
                               const std::vector<int32_t>& hardware_drive_ids) {
  auto log = Logger::getInstance("JointBinding");
  if (model_joint_names.empty()) {
    log->error("Configure failed: model joint list is empty");
    return Result::IllegalParameter;
  }
  if (hardware_drive_ids.empty()) {
    log->error("Configure failed: hardware drive id list is empty");
    return Result::IllegalParameter;
  }
  if (model_joint_names.size() > hardware_drive_ids.size()) {
    log->error("Configure failed: model joint count ({}) > hardware drive count ({})",
               model_joint_names.size(), hardware_drive_ids.size());
    return Result::IllegalParameter;
  }

  // model joint name 不能重复
  {
    std::set<std::string> unique_names(model_joint_names.begin(), model_joint_names.end());
    if (unique_names.size() != model_joint_names.size()) {
      log->error("Configure failed: duplicated model joint name in [{}]",
                 JoinStrings(model_joint_names));
      return Result::IllegalParameter;
    }
  }

  // hardware drive id 不能重复
  {
    std::set<int32_t> unique_ids(hardware_drive_ids.begin(), hardware_drive_ids.end());
    if (unique_ids.size() != hardware_drive_ids.size()) {
      log->error("Configure failed: duplicated hardware drive id in [{}]",
                 JoinDriveIds(hardware_drive_ids));
      return Result::IllegalParameter;
    }
  }

  model_joint_names_ = model_joint_names;
  hardware_drive_ids_ = hardware_drive_ids;
  joint_to_drive_id_.clear();
  log->info("Configured binding context: model joints={} [{}], hardware drives={} [{}]",
            model_joint_names_.size(), JoinStrings(model_joint_names_),
            hardware_drive_ids_.size(), JoinDriveIds(hardware_drive_ids_));
  return Result::NoError;
}

Result JointBinding::LoadFromYaml(const std::string& path) {
  auto log = Logger::getInstance("JointBinding");
  if (path.empty()) {
    log->error("LoadFromYaml failed: path is empty");
    return Result::IllegalParameter;
  }

  try {
    YAML::Node root = YAML::LoadFile(path);

    const std::string policy = root["binding_policy"].as<std::string>("explicit");
    if (policy != "explicit") {
      log->error("LoadFromYaml failed: unsupported binding_policy='{}' in {}",
                 policy, path);
      return Result::IllegalParameter;
    }

    const YAML::Node bindings = root["joint_bindings"];
    if (!bindings || !bindings.IsSequence()) {
      log->error("LoadFromYaml failed: missing or invalid 'joint_bindings' sequence in {}",
                 path);
      return Result::IllegalParameter;
    }

    joint_to_drive_id_.clear();
    for (const auto& entry : bindings) {
      const std::string joint_name = entry["joint_name"].as<std::string>("");
      if (joint_name.empty()) {
        log->error("LoadFromYaml failed: empty joint_name entry in {}", path);
        return Result::IllegalParameter;
      }

      if (!entry["drive_id"]) {
        log->error("LoadFromYaml failed: joint '{}' missing drive_id in {}",
                   joint_name, path);
        return Result::IllegalParameter;
      }
      const int32_t drive_id = entry["drive_id"].as<int32_t>();

      auto result = joint_to_drive_id_.emplace(joint_name, drive_id);
      if (!result.second) {
        log->error("LoadFromYaml failed: duplicated joint_name '{}' in {}",
                   joint_name, path);
        return Result::IllegalParameter;  // YAML 中 joint_name 重复
      }
    }

    auto rc = Validate();
    if (rc == Result::NoError) {
      log->info("Loaded joint binding YAML: path={}, entries={}",
                path, joint_to_drive_id_.size());
    } else {
      log->error("LoadFromYaml failed: validation failed after loading {}", path);
    }
    return rc;
  } catch (const YAML::Exception& e) {
    log->error("LoadFromYaml failed: YAML exception in {}: {}", path, e.what());
    return Result::ResourceUnavailable;
  } catch (const std::exception& e) {
    log->error("LoadFromYaml failed: exception in {}: {}", path, e.what());
    return Result::ResourceUnavailable;
  }
}

Result JointBinding::SaveToYaml(const std::string& path) const {
  auto log = Logger::getInstance("JointBinding");
  if (path.empty()) {
    log->error("SaveToYaml failed: path is empty");
    return Result::IllegalParameter;
  }

  YAML::Node root;
  root["binding_policy"] = "explicit";

  YAML::Node bindings;
  for (const auto& [joint_name, drive_id] : joint_to_drive_id_) {
    YAML::Node entry;
    entry["joint_name"] = joint_name;
    entry["drive_id"] = drive_id;
    bindings.push_back(entry);
  }
  root["joint_bindings"] = bindings;

  std::ofstream out(path);
  if (!out.is_open()) {
    log->error("SaveToYaml failed: could not open {}", path);
    return Result::ResourceUnavailable;
  }
  out << root;
  if (!out.good()) {
    log->error("SaveToYaml failed: write failed for {}", path);
    return Result::ResourceUnavailable;
  }
  log->info("Saved joint binding YAML: path={}, entries={}",
            path, joint_to_drive_id_.size());
  return Result::NoError;
}

Result JointBinding::Bind(const std::string& joint_name, int32_t drive_id) {
  auto log = Logger::getInstance("JointBinding");
  // 检查 joint_name 是否在已知列表中
  if (std::find(model_joint_names_.begin(), model_joint_names_.end(), joint_name)
      == model_joint_names_.end()) {
    log->error("Bind failed: unknown model joint '{}'", joint_name);
    return Result::IllegalParameter;
  }

  // 检查 drive_id 是否在已知列表中
  if (std::find(hardware_drive_ids_.begin(), hardware_drive_ids_.end(), drive_id)
      == hardware_drive_ids_.end()) {
    log->error("Bind failed: joint '{}' references unknown drive id {}",
               joint_name, drive_id);
    return Result::IllegalParameter;
  }

  joint_to_drive_id_[joint_name] = drive_id;
  log->info("Bound model joint '{}' -> drive id {}", joint_name, drive_id);
  return Result::NoError;
}

Result JointBinding::UnbindJoint(const std::string& joint_name) {
  joint_to_drive_id_.erase(joint_name);
  return Result::NoError;
}

void JointBinding::Clear() {
  joint_to_drive_id_.clear();
}

Result JointBinding::Validate() const {
  auto log = Logger::getInstance("JointBinding");
  // 绑定数量必须等于 model joint 数量
  if (joint_to_drive_id_.size() != model_joint_names_.size()) {
    log->error("Validate failed: binding count ({}) != model joint count ({})",
               joint_to_drive_id_.size(), model_joint_names_.size());
    return Result::IllegalParameter;
  }

  std::set<int32_t> used_drive_ids;

  for (const auto& joint_name : model_joint_names_) {
    auto it = joint_to_drive_id_.find(joint_name);
    // 每个 model joint 都必须有绑定
    if (it == joint_to_drive_id_.end()) {
      log->error("Validate failed: missing binding for model joint '{}'",
                 joint_name);
      return Result::IllegalParameter;
    }

    const int32_t drive_id = it->second;

    // drive_id 必须在已知列表中
    if (std::find(hardware_drive_ids_.begin(), hardware_drive_ids_.end(), drive_id)
        == hardware_drive_ids_.end()) {
      log->error("Validate failed: joint '{}' references unknown drive id {}",
                 joint_name, drive_id);
      return Result::IllegalParameter;
    }

    // drive_id 不能重复
    if (!used_drive_ids.insert(drive_id).second) {
      log->error("Validate failed: duplicated drive id {} in joint binding",
                 drive_id);
      return Result::IllegalParameter;
    }
  }

  log->info("Validated joint binding: {} model joints mapped", model_joint_names_.size());
  return Result::NoError;
}

std::vector<int32_t> JointBinding::GetModelIndexToDriveIds() const {
  std::vector<int32_t> result;
  result.reserve(model_joint_names_.size());
  for (const auto& joint_name : model_joint_names_) {
    auto it = joint_to_drive_id_.find(joint_name);
    result.push_back((it != joint_to_drive_id_.end()) ? it->second : -1);
  }
  return result;
}

int32_t JointBinding::DriveIdByModelIndex(size_t model_index) const {
  if (model_index >= model_joint_names_.size()) return -1;
  auto it = joint_to_drive_id_.find(model_joint_names_[model_index]);
  return (it != joint_to_drive_id_.end()) ? it->second : -1;
}

bool JointBinding::HasBindingForJoint(const std::string& joint_name) const {
  return joint_to_drive_id_.find(joint_name) != joint_to_drive_id_.end();
}

bool JointBinding::IsComplete() const {
  return joint_to_drive_id_.size() == model_joint_names_.size();
}

}  // namespace rocos
