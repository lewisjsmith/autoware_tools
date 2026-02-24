// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "route_history/yaml_storage.hpp"

#include "rclcpp/rclcpp.hpp"

#include <rclcpp/logging.hpp>

#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace autoware::route_history
{

void YamlStorage::set_path(const std::string & path)
{
  std::string expanded_path = expand_home_path(path);
  if (is_valid_file_path(expanded_path))
    path_ = expanded_path;
  else
    RCLCPP_ERROR(rclcpp::get_logger("yaml_storage"), "[set_path]: invalid file path.");
}

std::string YamlStorage::get_path()
{
  return path_;
}

std::vector<YAML::Node> YamlStorage::read()
{
  try {
    return YAML::LoadAllFromFile(path_);
  } catch (const YAML::BadFile & err) {
    RCLCPP_ERROR(rclcpp::get_logger("yaml_storage"), "[read]: %s.", err.what());
    return {};
  }
}

void YamlStorage::clear()
{
  std::ofstream o;
  o.open(path_, std::ios::trunc);
}

auto YamlStorage::expand_home_path(const std::string & path) -> std::string
{
  if (!path.empty() && path[0] == '~') {
    const char * home = getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

bool YamlStorage::is_valid_file_path(const std::string & path_str)
{
  std::filesystem::path path(path_str);
  std::filesystem::path parent = path.parent_path();

  if (!std::filesystem::exists(parent) || !std::filesystem::is_directory(parent)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("yaml_storage"),
      "[is_valid_file_path]: directory in save file path is not valid.");
    return false;
  }

  if (path_str.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("yaml_storage"), "[is_valid_file_path]: save file path is empty.");
    return false;
  }

  return true;
}

}  // namespace autoware::route_history
