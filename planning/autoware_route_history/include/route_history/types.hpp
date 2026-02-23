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

#ifndef ROUTE_HISTORY__TYPES_HPP_
#define ROUTE_HISTORY__TYPES_HPP_

#include "autoware_adapi_v1_msgs/msg/route.hpp"

#include <string>
#include <unordered_map>

namespace autoware::route_history
{

using adapi_route = autoware_adapi_v1_msgs::msg::Route;

struct NamedRoute
{
  std::string name;
  adapi_route route;
};

using uuid_route_map = std::unordered_map<std::string, NamedRoute>;

struct UuidName
{
  std::string uuid;
  std::string name;
};

}  // namespace autoware::route_history

#endif  // ROUTE_HISTORY__TYPES_HPP_
