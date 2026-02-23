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

#ifndef ROUTE_HISTORY__YAML_SAVE_HPP_
#define ROUTE_HISTORY__YAML_SAVE_HPP_

#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>
#include <vector>

namespace autoware::route_history
{

// Singleton?
class YamlManager
{
public:
  YamlManager() {}
  ~YamlManager() {}

  void set_path(const std::string & path) { _path = path; }
  std::string get_path() { return _path; }

  std::vector<YAML::Node> read()
  {
    std::vector<YAML::Node> data = YAML::LoadAllFromFile(_path);
    return data;
  }

  void clear()
  {
    std::ofstream o;
    o.open(_path, std::ios::trunc);
  }

  // clang-format off
  template<typename T>
  // clang-format on
  void write(T & value, bool append = true)
  {
    std::ofstream o;
    if (append) {
      o.open(_path, std::ios::app);
    } else {
      o.open(_path, std::ios::trunc);
    }

    if (!o.is_open()) {
      // RCLCPP_ERROR(node_->get_logger(), "[write_route] Cannot open file: %s.", filepath.c_str());
      throw std::runtime_error("[write_route] Cannot open file: " + _path + ".");
    }

    if (!o.is_open()) {
      throw std::runtime_error("[write_route] Cannot open file: " + _path + ".");
    }
    o << "---\n" << value << "\n";
    o.close();
  }

  // clang-format off
  template<typename T>
  // clang-format on
  void write(std::vector<T> & values, bool append = true)
  {
    std::ofstream o;
    if (append) {
      o.open(_path, std::ios::app);
    } else {
      o.open(_path, std::ios::trunc);
    }

    if (!o.is_open()) {
      // RCLCPP_ERROR(node_->get_logger(), "[write_route] Cannot open file: %s.", filepath.c_str());
      throw std::runtime_error("[write_route] Cannot open file: " + _path + ".");
    }

    if (!o.is_open()) {
      throw std::runtime_error("[write_route] Cannot open file: " + _path + ".");
    }
    for (auto & value : values) {
      o << "---\n" << value << "\n";
    }
    o.close();
  }

private:
  std::string _path = "~/.ros/route_history.yaml";
};
}  // namespace autoware::route_history

#endif  // ROUTE_HISTORY__YAML_SAVE_HPP_
