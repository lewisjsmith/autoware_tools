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

#ifndef ROUTE_HISTORY__YAML_STORAGE_HPP_
#define ROUTE_HISTORY__YAML_STORAGE_HPP_

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>
#include <vector>

namespace autoware::route_history
{

// Singleton?
class YamlStorage
{
public:
  YamlStorage() {}
  ~YamlStorage() {}

  static auto expand_home_path(const std::string &) -> std::string;
  static bool is_valid_file_path(const std::string &);

  void set_path(const std::string &);
  std::string get_path();

  std::vector<YAML::Node> read();

  void clear();

  template <typename T>
  void write(T & value, bool append = true)
  {
    std::ofstream o;
    if (append) {
      o.open(path_, std::ios::app);
    } else {
      o.open(path_, std::ios::trunc);
    }

    if (!o.is_open()) {
      throw std::runtime_error("[write_route] Cannot open file: " + path_ + ".");
    }

    o << "---\n" << value << "\n";
    o.close();
  }

  template <typename T>
  void write(std::vector<T> & values, bool append = true)
  {
    std::ofstream o;
    if (append) {
      o.open(path_, std::ios::app);
    } else {
      o.open(path_, std::ios::trunc);
    }

    if (!o.is_open()) {
      throw std::runtime_error("[write_route] Cannot open file: " + path_ + ".");
    }
    for (auto & value : values) {
      o << "---\n" << value << "\n";
    }
    o.close();
  }

  std::string path_;
};
}  // namespace autoware::route_history

#endif  // ROUTE_HISTORY__YAML_STORAGE_HPP_
