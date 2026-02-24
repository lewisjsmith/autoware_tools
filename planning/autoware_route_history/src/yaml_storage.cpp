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

#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>
#include <vector>

namespace autoware::route_history
{

void YamlStorage::set_path(const std::string & path)
{
  _path = expand_home_path(path);
}
std::string YamlStorage::get_path()
{
  return _path;
}

std::vector<YAML::Node> YamlStorage::read()
{
  try {
    return YAML::LoadAllFromFile(_path);
  } catch (const YAML::BadFile &) {
    return {};
  }
}

void YamlStorage::clear()
{
  std::ofstream o;
  o.open(_path, std::ios::trunc);
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

}  // namespace autoware::route_history
