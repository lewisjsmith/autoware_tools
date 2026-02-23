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

#ifndef ROUTE_HISTORY__YAML_UNMARSHAL_HPP_
#define ROUTE_HISTORY__YAML_UNMARSHAL_HPP_

#include "types.hpp"

#include "autoware_adapi_v1_msgs/msg/route.hpp"

#include <yaml-cpp/yaml.h>

#include <string>

namespace autoware::route_history
{

inline auto yaml_to_map(YAML::Node root) -> uuid_route_map
{
  autoware_adapi_v1_msgs::msg::Route msg;

  // Parse header
  auto header_node = root["header"];
  msg.header.stamp.sec = header_node["stamp"]["sec"].as<int32_t>();
  msg.header.stamp.nanosec = header_node["stamp"]["nanosec"].as<uint32_t>();
  msg.header.frame_id = header_node["frame_id"].as<std::string>();

  // Parse data (array, max 1)
  auto data_node = root["data"];
  if (data_node && data_node.IsSequence() && !data_node.IsNull()) {
    for (const auto & route_node : data_node) {
      autoware_adapi_v1_msgs::msg::RouteData route_data;

      // Start pose
      route_data.start.position.x = route_node["start"]["position"]["x"].as<double>();
      route_data.start.position.y = route_node["start"]["position"]["y"].as<double>();
      route_data.start.position.z = route_node["start"]["position"]["z"].as<double>();

      route_data.start.orientation.x = route_node["start"]["orientation"]["x"].as<double>();
      route_data.start.orientation.y = route_node["start"]["orientation"]["y"].as<double>();
      route_data.start.orientation.z = route_node["start"]["orientation"]["z"].as<double>();
      route_data.start.orientation.w = route_node["start"]["orientation"]["w"].as<double>();

      // Goal pose
      route_data.goal.position.x = route_node["goal"]["position"]["x"].as<double>();
      route_data.goal.position.y = route_node["goal"]["position"]["y"].as<double>();
      route_data.goal.position.z = route_node["goal"]["position"]["z"].as<double>();

      route_data.goal.orientation.x = route_node["goal"]["orientation"]["x"].as<double>();
      route_data.goal.orientation.y = route_node["goal"]["orientation"]["y"].as<double>();
      route_data.goal.orientation.z = route_node["goal"]["orientation"]["z"].as<double>();
      route_data.goal.orientation.w = route_node["goal"]["orientation"]["w"].as<double>();

      // Segments
      auto segments_node = route_node["segments"];
      if (segments_node && segments_node.IsSequence()) {
        for (const auto & seg_node : segments_node) {
          autoware_adapi_v1_msgs::msg::RouteSegment seg;

          // Preferred
          seg.preferred.id = seg_node["preferred"]["id"].as<int64_t>();
          seg.preferred.type = seg_node["preferred"]["type"].as<std::string>();

          // Alternatives
          auto alt_node = seg_node["alternatives"];
          if (alt_node && alt_node.IsSequence()) {
            for (const auto & alt : alt_node) {
              autoware_adapi_v1_msgs::msg::RoutePrimitive alt_prim;
              alt_prim.id = alt["id"].as<int64_t>();
              alt_prim.type = alt["type"].as<std::string>();
              seg.alternatives.push_back(alt_prim);
            }
          }

          route_data.segments.push_back(seg);
        }
      }

      msg.data.push_back(route_data);
    }
  }

  uuid_route_map map_obj = {
    {root["uuid"].as<std::string>(), {root["name"].as<std::string>(), msg}}};
  return map_obj;
}

}  // namespace autoware::route_history

#endif  // ROUTE_HISTORY__YAML_UNMARSHAL_HPP_
